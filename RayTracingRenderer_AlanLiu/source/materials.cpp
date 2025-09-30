#include "materials.h"
#include "lights.h"



Color MtlPhong::Shade(ShadeInfo const& shadeInfo) const
{
	const Vec3f N = shadeInfo.N().GetNormalized();		// normal
	const Vec3f V = shadeInfo.V().GetNormalized();		// viewing
	Color color(0.0f, 0.0f, 0.0f);

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
			color += Diffuse() * lightColor;
			continue;
		}

		const float NdotL = std::max(0.0f, N % L);
		if (NdotL > 0.0f)
		{
			// Add diffuse
			color += Diffuse() * lightColor * NdotL;

			Vec3f R = (2.0f * NdotL) * N - L;		// Reflection
			R.Normalize();
			const float RdotV = std::max(0.0f, R % V);
			// Add specular
			color += Specular() * lightColor * std::pow(RdotV, Glossiness());
		}
	}

	// Saturate the color
	color.Clamp();

	return color;
}


Color MtlBlinn::Shade(ShadeInfo const& shadeInfo) const
{
	const Vec3f N = shadeInfo.N().GetNormalized();		// normal
	const Vec3f V = shadeInfo.V().GetNormalized();		// viewing
	Color color(0.0f, 0.0f, 0.0f);

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
			color += Diffuse() * lightColor;
			continue;
		}

		const float NdotL = std::max(0.0f, N % L);
		if (NdotL > 0.0f)
		{
			// Add diffuse
			color += Diffuse() * lightColor * NdotL;
			Vec3f H = (L + V).GetNormalized();		// halfway vector H
			const float NdotH = std::max(0.0f, N % H);
			// Add Specular
			color += Specular() * lightColor * std::pow(NdotH, Glossiness());
		}
	}

	// Saturate the color
	color.Clamp();

	return color;
}


Color MtlMicrofacet::Shade(ShadeInfo const& shadeInfo) const
{
	return Color(0, 0, 0);
}