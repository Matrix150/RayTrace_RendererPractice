#include <iostream>
#define NOMINMAX
#include <windows.h>

#include "scene.h"
#include "renderer.h"

Renderer* CreateRenderer();


// Choose xml file to open
static bool OpenXmlFile(std::string& outPath)
{
    CHAR fileBuf[MAX_PATH] = { 0 };

    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFilter = "XML Files (*.xml)\0*.xml\0All Files (*.*)\0*.*\0";
    ofn.nFilterIndex = 1;
    ofn.lpstrFile = fileBuf;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrTitle = "Select XML scene file";
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_EXPLORER;

    if (GetOpenFileNameA(&ofn)) {
        outPath.assign(fileBuf);
        return true;
    }
    return false;
}

int main() {
    Renderer* renderer = CreateRenderer();

    // Select xml
    std::string xmlPath;
    if (!OpenXmlFile(xmlPath))
    {
        std::cout << "No file selected.\n";
        delete renderer;
        return 0;
    }

    // Load scene
    if (!renderer->LoadScene(xmlPath.c_str()))
    {
        std::cout << "Failed to load xml file" << xmlPath << std::endl;
        delete renderer;
        return -1;
    }

    // Open Viewport
    ShowViewport(renderer, false);

    delete renderer;
    return 0;
}