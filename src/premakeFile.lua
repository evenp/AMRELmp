
SrcDir = os.getcwd()
UsingQT = false;

function includeShapeLib()
	includedirs(SrcDir.."/../deps/shapelib")
	filter { "system:not windows" }
		links { "shp" }
	filter { "system:Windows", "configurations:Debug" }
		libdirs(SrcDir.."/../deps/shapelib/build/dll/Debug")
		links { "shp.lib" }
	filter { "system:Windows", "configurations:Release" }
		libdirs(SrcDir.."/../deps/shapelib/build/dll/Release")
		links { "shp.lib" }
	filter { }
end

function includeStbi()
	includedirs(SrcDir.."/../src/Libs/stbi")
end

workspace "AMREL"
	configurations { "Debug", "Release" }
	startproject "AMREL"
	architecture "x86_64"
	location (SrcDir.."/../")

project "AMREL"
	--project configuration
	kind ("ConsoleApp")
	language "C++"
	cppdialect "C++17"
	files { "**.cpp", "**.hpp", "**.h", "**.c", "**.cxx" }

	--vs paths
	targetdir (SrcDir.."/../binaries/".."%{prj.name}".."/".."%{cfg.longname}")
	objdir (SrcDir.."/../intermediate/".."%{prj.name}".."/".."%{cfg.longname}")
	debugdir(SrcDir.."/../resources")

	filter "configurations:Debug"
		defines { "DEBUG" }
		symbols "On"
	filter "configurations:Release"
		defines { "NDEBUG" }
		optimize "On"
	filter { }

	filter "system:windows"
		buildoptions { "/Ot", "/MP" }
	filter { }

	--Includes
	includedirs(SrcDir.."/Amrel")
	includedirs(SrcDir.."/ASDetector")
	includedirs(SrcDir.."/BlurredSegment")
	includedirs(SrcDir.."/DirectionalScanner")
	includedirs(SrcDir.."/ImageTools")
	includedirs(SrcDir.."/PointCloud")
	includeShapeLib()
	includeStbi()
