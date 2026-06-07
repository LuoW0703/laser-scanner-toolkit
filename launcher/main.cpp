#include <windows.h>
#include <string>

int WINAPI wWinMain(HINSTANCE, HINSTANCE, PWSTR, int) {
    wchar_t modulePath[MAX_PATH] = {};
    const DWORD length = GetModuleFileNameW(nullptr, modulePath, MAX_PATH);
    if (length == 0 || length == MAX_PATH) {
        MessageBoxW(nullptr, L"Unable to locate the application directory.",
                    L"Laser Scanner Toolkit", MB_OK | MB_ICONERROR);
        return 1;
    }

    std::wstring root(modulePath, length);
    const std::wstring::size_type separator = root.find_last_of(L"\\/");
    if (separator == std::wstring::npos) {
        MessageBoxW(nullptr, L"Unable to locate the application directory.",
                    L"Laser Scanner Toolkit", MB_OK | MB_ICONERROR);
        return 1;
    }
    root.resize(separator);

    const std::wstring appDir = root + L"\\app";
    const std::wstring application = appDir + L"\\LaserScannerToolkit.exe";

    if (GetFileAttributesW(application.c_str()) == INVALID_FILE_ATTRIBUTES) {
        MessageBoxW(nullptr,
                    L"The application package is incomplete.\n"
                    L"Expected: app\\LaserScannerToolkit.exe",
                    L"Laser Scanner Toolkit", MB_OK | MB_ICONERROR);
        return 2;
    }

    STARTUPINFOW startupInfo = {};
    startupInfo.cb = sizeof(startupInfo);
    PROCESS_INFORMATION processInfo = {};
    std::wstring commandLine = L"\"" + application + L"\"";

    const BOOL started = CreateProcessW(
        application.c_str(), commandLine.data(), nullptr, nullptr, FALSE, 0,
        nullptr, appDir.c_str(), &startupInfo, &processInfo);
    if (!started) {
        MessageBoxW(nullptr, L"Failed to start Laser Scanner Toolkit.",
                    L"Laser Scanner Toolkit", MB_OK | MB_ICONERROR);
        return 3;
    }

    CloseHandle(processInfo.hThread);
    CloseHandle(processInfo.hProcess);
    return 0;
}
