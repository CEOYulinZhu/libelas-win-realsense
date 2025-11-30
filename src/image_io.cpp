#include "image.h"

#ifdef _WIN32

// Windows 专用实现：使用 Windows Imaging Component (WIC)
// 支持从 PNG / JPG 等常见格式读取图像，并统一转换为 8bit 灰度图像。
// 这样可以在不改变 libelas 主体算法的前提下，扩展输入图像格式。

#include <windows.h>
#include <wincodec.h>
#include <string>
#include <iostream>
#include <cstring>

// 辅助：将窄字符路径转换为宽字符（ANSI -> UTF-16），便于 WIC 使用
static bool AnsiToWide(const char* src, std::wstring& dst) {
    if (!src) return false;
    int len = MultiByteToWideChar(CP_ACP, 0, src, -1, nullptr, 0);
    if (len <= 0) return false;
    dst.resize(static_cast<size_t>(len - 1));
    if (len > 1) {
        MultiByteToWideChar(CP_ACP, 0, src, -1, &dst[0], len);
    }
    return true;
}

// 辅助：根据扩展名（不区分大小写）判断文件是否为指定后缀，例如 .pgm / .png / .jpg
static bool HasExtension(const char* filename, const char* ext) {
    if (!filename || !ext) return false;
    const char* dot = std::strrchr(filename, '.');
    if (!dot) return false;
#ifdef _MSC_VER
    return _stricmp(dot, ext) == 0;
#else
    return strcasecmp(dot, ext) == 0;
#endif
}

// 通用图像读入接口：
// - .pgm 仍然走原有的 loadPGM；
// - 其他扩展名（如 .png/.jpg）在 Windows 下通过 WIC 加载并转为 8bit 灰度。
image<uchar>* loadImage(const char* name) {
    // 1) 保持对原 PGM 流程的兼容
    if (HasExtension(name, ".pgm")) {
        return loadPGM(name);
    }

    // 2) Windows 下使用 WIC 读取 PNG / JPG 等格式
    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    bool coInitialized = (hr == S_OK || hr == S_FALSE);

    IWICImagingFactory* factory = nullptr;
    IWICBitmapDecoder* decoder = nullptr;
    IWICBitmapFrameDecode* frame = nullptr;
    IWICFormatConverter* converter = nullptr;
    image<uchar>* resultImg = nullptr;

    std::wstring wname;
    if (!AnsiToWide(name, wname)) {
        std::cerr << "ERROR: Failed to convert path to wide char: " << name << std::endl;
        goto Cleanup;
    }

    hr = CoCreateInstance(
        CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
        IID_PPV_ARGS(&factory));
    if (FAILED(hr)) {
        std::cerr << "ERROR: CoCreateInstance(WICImagingFactory) failed, hr=0x"
                  << std::hex << hr << std::dec << std::endl;
        goto Cleanup;
    }

    hr = factory->CreateDecoderFromFilename(
        wname.c_str(), nullptr, GENERIC_READ,
        WICDecodeMetadataCacheOnLoad, &decoder);
    if (FAILED(hr)) {
        std::cerr << "ERROR: Failed to open image with WIC: " << name << std::endl;
        goto Cleanup;
    }

    hr = decoder->GetFrame(0, &frame);
    if (FAILED(hr)) {
        std::cerr << "ERROR: Failed to get WIC frame for: " << name << std::endl;
        goto Cleanup;
    }

    hr = factory->CreateFormatConverter(&converter);
    if (FAILED(hr)) {
        std::cerr << "ERROR: CreateFormatConverter failed for: " << name << std::endl;
        goto Cleanup;
    }

    // 将任意输入格式转换为 8bit 灰度格式 GUID_WICPixelFormat8bppGray
    hr = converter->Initialize(
        frame,
        GUID_WICPixelFormat8bppGray,
        WICBitmapDitherTypeNone,
        nullptr,
        0.0,
        WICBitmapPaletteTypeCustom);
    if (FAILED(hr)) {
        std::cerr << "ERROR: Failed to convert image to 8bpp gray: " << name << std::endl;
        goto Cleanup;
    }

    UINT w = 0, h = 0;
    hr = converter->GetSize(&w, &h);
    if (FAILED(hr) || w == 0 || h == 0) {
        std::cerr << "ERROR: Invalid image size when loading: " << name << std::endl;
        goto Cleanup;
    }

    // 为 libelas 分配内部使用的灰度图像（8bit）
    resultImg = new image<uchar>(static_cast<int>(w), static_cast<int>(h));
    const UINT stride = w;              // 8bit 灰度: 每行 w 字节
    const UINT bufferSize = stride * h; // 总字节数

    hr = converter->CopyPixels(
        nullptr,
        stride,
        bufferSize,
        resultImg->data);
    if (FAILED(hr)) {
        std::cerr << "ERROR: CopyPixels failed when loading: " << name << std::endl;
        delete resultImg;
        resultImg = nullptr;
        goto Cleanup;
    }

Cleanup:
    if (converter) converter->Release();
    if (frame) frame->Release();
    if (decoder) decoder->Release();
    if (factory) factory->Release();
    if (coInitialized) CoUninitialize();

    // 如果 WIC 流程失败，则尝试按 PGM 再读一次；若仍失败则返回 nullptr
    if (!resultImg) {
        try {
            return loadPGM(name);
        } catch (...) {
            std::cerr << "ERROR: Failed to load image as PNG/JPG or PGM: " << name << std::endl;
            return nullptr;
        }
    }

    return resultImg;
}

// 将 8 位灰度图保存为 PNG（使用 WIC）
void savePNG(image<uchar>* im, const char* name) {
    if (!im || !name) {
        std::cerr << "ERROR: savePNG invalid arguments." << std::endl;
        return;
    }

    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    bool coInitialized = (hr == S_OK || hr == S_FALSE);

    IWICImagingFactory* factory = nullptr;
    IWICStream* stream = nullptr;
    IWICBitmapEncoder* encoder = nullptr;
    IWICBitmapFrameEncode* frame = nullptr;
    IPropertyBag2* props = nullptr;

    std::wstring wname;
    if (!AnsiToWide(name, wname)) {
        std::cerr << "ERROR: Failed to convert output path to wide char: " << name << std::endl;
        goto Cleanup;
    }

    hr = CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factory));
    if (FAILED(hr)) {
        std::cerr << "ERROR: CoCreateInstance(WICImagingFactory) failed for savePNG, hr=0x" << std::hex << hr << std::dec << std::endl;
        goto Cleanup;
    }

    hr = factory->CreateStream(&stream);
    if (FAILED(hr)) { std::cerr << "ERROR: CreateStream failed." << std::endl; goto Cleanup; }
    hr = stream->InitializeFromFilename(wname.c_str(), GENERIC_WRITE);
    if (FAILED(hr)) { std::cerr << "ERROR: InitializeFromFilename for output failed." << std::endl; goto Cleanup; }

    hr = factory->CreateEncoder(GUID_ContainerFormatPng, nullptr, &encoder);
    if (FAILED(hr)) { std::cerr << "ERROR: CreateEncoder(PNG) failed." << std::endl; goto Cleanup; }
    hr = encoder->Initialize(stream, WICBitmapEncoderNoCache);
    if (FAILED(hr)) { std::cerr << "ERROR: Encoder Initialize failed." << std::endl; goto Cleanup; }

    hr = encoder->CreateNewFrame(&frame, &props);
    if (FAILED(hr)) { std::cerr << "ERROR: CreateNewFrame failed." << std::endl; goto Cleanup; }
    hr = frame->Initialize(props);
    if (FAILED(hr)) { std::cerr << "ERROR: Frame Initialize failed." << std::endl; goto Cleanup; }

    const UINT w = static_cast<UINT>(im->width());
    const UINT h = static_cast<UINT>(im->height());
    hr = frame->SetSize(w, h);
    if (FAILED(hr)) { std::cerr << "ERROR: SetSize failed." << std::endl; goto Cleanup; }

    WICPixelFormatGUID fmt = GUID_WICPixelFormat8bppGray;
    hr = frame->SetPixelFormat(&fmt);
    if (FAILED(hr)) { std::cerr << "ERROR: SetPixelFormat failed." << std::endl; goto Cleanup; }
    // 如果编码器不支持 8bppGray，会改写 fmt；此时为简单起见直接报错
    if (!IsEqualGUID(fmt, GUID_WICPixelFormat8bppGray)) {
        std::cerr << "ERROR: PNG encoder does not support 8bpp gray directly." << std::endl;
        goto Cleanup;
    }

    const UINT stride = w;              // 8bit 灰度，每行 w 字节
    const UINT bufferSize = stride * h; // 总字节数
    hr = frame->WritePixels(h, stride, bufferSize, im->data);
    if (FAILED(hr)) { std::cerr << "ERROR: WritePixels failed." << std::endl; goto Cleanup; }

    hr = frame->Commit();
    if (FAILED(hr)) { std::cerr << "ERROR: Frame Commit failed." << std::endl; goto Cleanup; }
    hr = encoder->Commit();
    if (FAILED(hr)) { std::cerr << "ERROR: Encoder Commit failed." << std::endl; goto Cleanup; }

Cleanup:
    if (props) props->Release();
    if (frame) frame->Release();
    if (encoder) encoder->Release();
    if (stream) stream->Release();
    if (factory) factory->Release();
    if (coInitialized) CoUninitialize();
}

#else  // 非 Windows 平台：保持兼容，只支持 PGM

// 在非 Windows 平台上不引入额外依赖，loadImage 仅简单调用原有的 loadPGM。
image<uchar>* loadImage(const char* name) {
    return loadPGM(name);
}

// 非 Windows 平台：退化处理，仍保存为 PGM（若被调用）
void savePNG(image<uchar>* im, const char* name) {
    (void)name; // unused
    std::cerr << "WARNING: savePNG is not supported on this platform; saving PGM instead." << std::endl;
    // 将 .png 扩展替换为 .pgm 简单保存
    char out[1024];
    const char* dot = std::strrchr(name, '.');
    size_t len = dot ? (size_t)(dot - name) : std::strlen(name);
    if (len > sizeof(out) - 5) len = sizeof(out) - 5;
    std::memcpy(out, name, len); out[len] = '\0'; std::strcat(out, ".pgm");
    savePGM(im, out);
}

#endif
