#include "materials.h"
#include "lights.h"



static inline float Saturate(float x)
{
	return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

static inline float FresnelSchlick(float cosTheta, float nIn, float nOut)
{
	cosTheta = std::fabs(cosTheta);
	const float F0 = std::pow((nIn - nOut) / (nIn + nOut), 2.0f);

	return F0 + (1.0f - F0) * std::pow(1.0f - cosTheta, 5.0f);
}

static inline Vec3f Reflect(const Vec3f& V, const Vec3f& N)
{
	return (2.0f * (N % V)) * N - V;
}

static inline bool Refract(const Vec3f& V, const Vec3f& N_geo, bool isFront, float ior, Vec3f& T_out, float& cosi_out, float& eta_out)
{
	float cosi = isFront ? (V % N_geo) : -(V % N_geo);
	Vec3f N = isFront ? N_geo : -(N_geo);
	const float nIn = isFront ? 1.0f : ior;
	const float nOut = isFront ? ior : 1.0f;

	const float eta = nIn / nOut;
	const float k = 1.0f - eta * eta * (1.0f - cosi * cosi);

	if (k < 0.0f)
		return false;		// Total internal reflection

	const float cost = std::sqrt(k);
	T_out = ((-eta) * V - (cost - eta * cosi) * N).GetNormalized();
	cosi_out = cosi;
	eta_out = eta;
	return true;
}

static inline Color Absorb(Color const& sigma, float dist)
{
	return Color(std::expf(-sigma.r * dist), std::expf(-sigma.g * dist), std::expf(-sigma.b * dist));
}

static inline void BuildTBN(const Vec3f& N, Vec3f& T, Vec3f& B)
{
	Vec3f up = (std::fabs(N.z) < 0.999f) ? Vec3f(0.0f, 0.0f, 1.0f) : Vec3f(0.0f, 1.0f, 0.0f);
	T = (up ^ N).GetNormalized();
	B = (N ^ T).GetNormalized();
}

static inline Vec3f SampleAroundAxis(const Vec3f& idealDir, float alpha, float u1, float u2)
{
	float cosTheta = std::pow(u1, 1.0f / (alpha + 1.0f));
	float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
	float phi = 2.0f * Pi<float>() * u2;

	Vec3f T, B;
	BuildTBN(idealDir, T, B);

	Vec3f w = (T * (sinTheta * std::cos(phi)) + B * (sinTheta * std::sin(phi)) + idealDir * cosTheta).GetNormalized();
	return w;
}

static Vec3f SampleHemisphereUniform(const Vec3f& N, float u1, float u2, float& pdf)
{
	float cosTheta = 1.0f - u1;
	float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
	float phi = 2.0f * Pi<float>() * u2;

	Vec3f T, B;
	BuildTBN(N, T, B);
	Vec3f w = (T * (sinTheta * std::cos(phi)) + B * (sinTheta * std::sin(phi)) + N * cosTheta).GetNormalized();

	pdf = 1.0f / (2.0f * Pi<float>());
	return w;
}

static Vec3f SampleHemisphereCosine(const Vec3f& N, float u1, float u2, float& pdf)
{
	float cosTheta = std::sqrt(1.0f - u1);
	float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
	float phi = 2.0f * Pi<float>() * u2;
	
	Vec3f T, B;
	BuildTBN(N, T, B);
	Vec3f w = (T * (sinTheta * std::cos(phi)) + B * (sinTheta * std::sin(phi)) + N * cosTheta).GetNormalized();

	pdf = cosTheta / Pi<float>();
	return w;
}


// Shade model
Color MtlPhong::Shade(ShadeInfo const& shadeInfo) const
{
	const Vec3f N = shadeInfo.N().GetNormalized();		// normal
	const Vec3f V = shadeInfo.V().GetNormalized();		// viewing
	Color color(0.0f, 0.0f, 0.0f);

	const Color diffuse = shadeInfo.Eval(Diffuse());
	const Color specular = shadeInfo.Eval(Specular());
	const float glossiness = shadeInfo.Eval(Glossiness());

	const int lightNum = shadeInfo.NumLights();	// light numbers
	// Loop all lights
	for (int i = 0; i < lightNum; ++i)
	{
		const Light* light = shadeInfo.GetLight(i);
		Vec3f L;
		Color lightColor = light->Illuminate(shadeInfo, L);		// Get light intensity and direction(vector L)
		L.Normalize();

		// Add ambient light
		if (light->IsAmbient())
		{
			color += diffuse * lightColor;
			continue;
		}

		const float NdotL = std::max(0.0f, N % L);
		if (NdotL > 0.0f)
		{
			// Add diffuse
			color += diffuse * lightColor * NdotL;

			Vec3f R = (2.0f * NdotL) * N - L;		// Reflection
			R.Normalize();
			const float RdotV = std::max(0.0f, R % V);
			// Add specular
			color += specular * lightColor * std::pow(RdotV, glossiness);
		}
	}

	// Saturate the color
	//color.Clamp();

	return color;
}


Color MtlBlinn::Shade(ShadeInfo const& sInfo) const
{
	const Vec3f N = sInfo.N().GetNormalized();		// Normal
	const Vec3f V = sInfo.V().GetNormalized();		// Viewing
	const Vec3f P = sInfo.P();
	const bool front = sInfo.IsFront();
	Color color(0.0f, 0.0f, 0.0f);

	// Texture
	const Color diffuseTex = sInfo.Eval(Diffuse());
	const Color specularTex = sInfo.Eval(Specular());
	const float glossiness = sInfo.Eval(Glossiness());

	// Reflection & Refraction
	const Color kr = sInfo.Eval(Reflection());
	const Color kt = sInfo.Eval(Refraction());
	const bool hasKt = (kt.r > 0.0f || kt.g > 0.0f || kt.b > 0.0f);
	const bool hasKr = (kr.r > 0.0f || kr.g > 0.0f || kr.b > 0.0f) || hasKt;

	// Energy-conserving
	Color ks = specularTex;
	ks.Clamp();
	float ksMax = std::max(ks.r, std::max(ks.g, ks.b));
	ksMax = std::min(ksMax, 1.0f);
	float kdScale = std::max(0.0f, 1.0f - ksMax);
	Color kd = diffuseTex * kdScale;

	const int lightNum = sInfo.NumLights();	// Light numbers
	// Loop all lights
	for (int i = 0; i < lightNum; ++i)
	{
		const Light* light = sInfo.GetLight(i);
		Vec3f L;
		Color lightColor = light->Illuminate(sInfo, L);		// Get light intensity and direction(vector L)
		L.Normalize();

		// Add ambient light
		if (light->IsAmbient())
		{
			color += kd * lightColor;
			continue;
		}

		const float NdotL = std::max(0.0f, N % L);
		if (NdotL > 0.0f)
		{
			// Diffuse
			color += kd * lightColor * (NdotL / Pi<float>());
			// Blinn-Phong specular
			if (!hasKr && !hasKt)
			{
				Vec3f H = (L + V).GetNormalized();		// halfway vector H
				const float NdotH = std::max(0.0f, N % H);
				const float specNorm = (glossiness + 2.0f) / (8.0f * Pi<float>());
				const float specular = specNorm * std::pow(NdotH, glossiness);
				color += ks * lightColor * specular;
			}
		}
	}

	const bool canBounce = sInfo.CanBounce();
	// Inderect diffuse Global Illumination (Monte Carlo)
	if (canBounce && sInfo.CurrentBounce() <= 1)
	{
		Color indirect(0.0f, 0.0f, 0.0f);
		const int numSample = 32;
		const float shift1 = sInfo.RandomFloat();
		const float shift2 = sInfo.RandomFloat();
		constexpr int haltonOffset = 7;

		auto frac = [](float x) {return x - std::floor(x); };

		for (int i = 0; i < numSample; ++i)
		{
			int index = i + haltonOffset;
			float u1 = frac(Halton(index + 1, 2) + shift1);
			float u2 = frac(Halton(index + 1, 3) + shift2);
			float pdf = 0.0f;
			Vec3f wi = SampleHemisphereCosine(N, u1, u2, pdf);

			const float NdotWi = std::max(0.0f, N % wi);
			if (NdotWi <= 0.0f || pdf <= 0)
				continue;

			Ray giRay(P + N * 1e-4f, wi);
			float dist = BIGFLOAT;

			Color giDiffuse = sInfo.TraceSecondaryRay(giRay, dist, false);
			Color brdf = kd * (1.0f / Pi<float>());
			indirect += brdf * giDiffuse * (NdotWi / pdf);
		}

		indirect /= static_cast<float>(numSample);
		color += indirect;
	}

	// Specular & Transmission
	if (canBounce)
	{
		constexpr float kEps = 1e-4f;
		const Color sigma = Absorption();
		const float ior = IOR();

		const float nIn = front ? 1.0f : ior;
		const float	nOut = front ? ior : 1.0f;
		float cosi = std::fabs(N % V);
		const float F = FresnelSchlick(cosi, nIn, nOut);

		const float reflectionWeight = hasKt ? F : 1.0f;
		const float refractionWeight = hasKt ? (1.0f - F) : 0.0f;

		Vec3f Tdir;
		float eta = 1.0f;
		bool refractionDone = false;

		auto frac = [](float x) {return x - std::floor(x); };

		const float refrShift1 = sInfo.RandomFloat();
		const float refrShift2 = sInfo.RandomFloat();
		constexpr int refrHaltonOffset = 23;
		const float reflShift1 = sInfo.RandomFloat();
		const float reflShift2 = sInfo.RandomFloat();
		constexpr int reflHaltonOffset = 47;


		if (hasKt && refractionWeight > 0.0f)
		{
			refractionDone = Refract(V, N, front, ior, Tdir, cosi, eta);
			// Refraction
			if (refractionDone)
			{
				const bool entering = (nIn < nOut);		// Air into object
				Vec3f nOffset = entering ? (-N) : N;

				int nSampleT = 1;
				if (glossiness < 8.0f)
					nSampleT = 16;
				else if (glossiness < 32.0f)
					nSampleT = 8;
				else if (glossiness < 128.0f)
					nSampleT = 4;

				Color LtAccum(0.0f, 0.0f, 0.0f);
				for (int s = 0; s < nSampleT; ++s)
				{
					int index = s + refrHaltonOffset;
					float u1 = frac(Halton(index + 1, 2) + refrShift1);
					float u2 = frac(Halton(index + 1, 3) + refrShift2);
					Vec3f Tsample = SampleAroundAxis(Tdir, glossiness, u1, u2);

					Ray tRay(P + nOffset * kEps, Tsample);
					float distT = BIGFLOAT;
					Color Lt = sInfo.TraceSecondaryRay(tRay, distT, false);

					if (entering)
						Lt *= Absorb(sigma, distT);
					LtAccum += Lt;
				}
				LtAccum /= (float)nSampleT;

				color += kt * refractionWeight * LtAccum;
			}
		}

		// Reflection or Total Internal Refraction
		if (hasKr && reflectionWeight > 0.0f)
		{
			Vec3f Rdir = Reflect(V, N).GetNormalized();

			int nSampleR = 1;
			if (glossiness < 8.0f)
				nSampleR = 16;
			else if (glossiness < 32.0f)
				nSampleR = 8;
			else if (glossiness < 128.0f)
				nSampleR = 4;

			Color LrAccum(0.0f, 0.0f, 0.0f);
			for (int s = 0; s < nSampleR; ++s)
			{
				Vec3f Rsample;
				int guard = 0;

				do
				{
					int index = s + reflHaltonOffset;
					float u1 = frac(Halton(index + 1, 2) + reflShift1);
					float u2 = frac(Halton(index + 1, 3) + reflShift2);
					Rsample = SampleAroundAxis(Rdir, glossiness, u1, u2);
					++guard;
				} while ((N % Rsample) <= 0.0f && guard < 16);

				if (N % Rsample <= 0.0f)
					continue;

				Vec3f nOffsetR = front ? N : -N;
				Ray rRay(P + nOffsetR * kEps, Rsample);
				float distR = BIGFLOAT;
				Color Lr = sInfo.TraceSecondaryRay(rRay, distR, true);

				LrAccum += Lr;
			}
			LrAccum /= (float)nSampleR;
			
			color += kr * reflectionWeight * LrAccum;
		}
	}

	color += sInfo.Eval(Emission());

	// Saturate the color
	//color.Clamp();

	return color;
}


Color MtlMicrofacet::Shade(ShadeInfo const& shadeInfo) const
{
	return Color(0, 0, 0);
}