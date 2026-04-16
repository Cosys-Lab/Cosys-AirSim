// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef common_utils_FileSystem_hpp
#define common_utils_FileSystem_hpp

#include <filesystem>
#include <fstream>
#include <string>
#include "Utils.hpp"

// This defines a default folder name for all the files created by AirLib so they
// are all gathered nicely in one place in the user's documents folder.
#ifndef ProductFolderName
#define ProductFolderName "AirSim"
#endif

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
#endif

namespace common_utils
{
class FileSystem
{
    typedef unsigned int uint;

public:
    static const char kPathSeparator =
#ifdef _WIN32
        '\\';
#else
        '/';
#endif

    static std::string createDirectory(const std::string& fullPath)
    {
        std::filesystem::path fs_path(fullPath);
        if (!std::filesystem::exists(fs_path)) {
            std::filesystem::create_directories(fs_path);
        }
        return fullPath;
    }

    static std::string getUserHomeFolder()
    {
#ifdef _WIN32
        std::wstring userProfile = _wgetenv(L"USERPROFILE");
        return std::filesystem::path(userProfile).u8string();
#else
        return std::getenv("HOME");
#endif
    }

    static std::string getUserDocumentsFolder()
    {
#ifdef _WIN32
        std::wstring userProfile = _wgetenv(L"USERPROFILE");
        return (std::filesystem::path(userProfile) / "Documents").u8string();
#else
        return std::getenv("HOME");
#endif
    }

    static std::string getExecutableFolder()
    {
        return std::filesystem::current_path().u8string();
    }

    static std::string getAppDataFolder()
    {
        return ensureFolder(combine(getUserDocumentsFolder(), ProductFolderName));
    }

    static std::string ensureFolder(const std::string& fullpath)
    {
        return createDirectory(fullpath);
    }

    static std::string ensureFolder(const std::string& parentFolder, const std::string& child)
    {
        return createDirectory(combine(parentFolder, child));
    }

    static std::string combine(const std::string& parentFolder, const std::string& child)
    {
        if (child.size() == 0)
            return parentFolder;

        std::filesystem::path p(parentFolder);
        p /= child;
        return p.u8string();
    }

    static void removeLeaf(std::string& path)
    {
        std::filesystem::path p(path);
        if (p.has_parent_path()) {
            path = p.parent_path().u8string();
        }
    }

    static std::string getFileExtension(const std::string& str)
    {
        std::filesystem::path p(str);
        if (p.has_extension()) {
            return p.extension().u8string();
        }
        return "";
    }

    static std::string getLogFolderPath(bool folder_timestamp, const std::string& parent = "")
    {
        std::string logfolder = folder_timestamp ? Utils::to_string(Utils::now()) : "";
        std::string parent_folder = (parent == "") ? getAppDataFolder() : parent;
        std::string fullPath = combine(parent_folder, logfolder);
        ensureFolder(fullPath);

        return fullPath;
    }

    static std::string getLogFileNamePath(const std::string& fullPath, const std::string& prefix, const std::string& suffix, const std::string& extension,
                                              bool file_timestamp)
    {
        std::string filename;
        filename.append(ensureFolder(fullPath))
            .push_back(kPathSeparator);
        filename.append(prefix)
            .append(suffix)
            .append(file_timestamp ? Utils::to_string(Utils::now()) : "")
            .append(extension);

        return filename;
    }

    static void openTextFile(const std::string& filepath, std::ifstream& file)
    {
        std::filesystem::path fs_path(filepath);
        file.open(fs_path, std::ios::in);
    }

    static void createBinaryFile(const std::string& filepath, std::ofstream& file)
    {
        std::filesystem::path fs_path(filepath);
        file.open(fs_path, std::ios::binary | std::ios::trunc);
    }

    static void createTextFile(const std::string& filepath, std::ofstream& file)
    {
        std::filesystem::path fs_path(filepath);
        file.open(fs_path, std::ios::out | std::ios::trunc);

        if (file.fail())
            throw std::ios_base::failure(std::strerror(errno));
    }

    static std::string createLogFile(const std::string& suffix, std::ofstream& flog)
    {
        std::string log_folderpath = common_utils::FileSystem::getLogFolderPath(false);
        std::string filepath = getLogFileNamePath(log_folderpath, "log_", suffix, ".tsv", true);
        createTextFile(filepath, flog);

        Utils::log(Utils::stringf("log file started: %s", filepath.c_str()));
        flog.exceptions(flog.exceptions() | std::ios::failbit | std::ifstream::badbit);
        return filepath;
    }

    static std::string readLineFromFile(std::ifstream& file)
    {
        std::string line;
        try {
            std::getline(file, line);
        }
        catch (...) {
            if (!file.eof())
                throw;
        }
        return line;
    }

    static void appendLineToFile(const std::string& filepath, const std::string& line)
    {
        std::ofstream file;
        std::filesystem::path fs_path(filepath);
        file.open(fs_path, std::ios::out | std::ios::app);
        if (file.fail())
            throw std::ios_base::failure(std::strerror(errno));
        file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);
        file << line << std::endl;
    }
};
}
#endif