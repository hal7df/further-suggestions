<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>
<CodeBlocks_project_file>
	<FileVersion major="1" minor="6" />
	<Project>
		<Option title="Lessons" />
		<Option pch_mode="2" />
		<Option compiler="gcc" />
		<Build>
			<Target title="Debug">
				<Option output="bin/Debug/Lessons" prefix_auto="1" extension_auto="1" />
				<Option object_output="obj/Debug/" />
				<Option type="1" />
				<Option compiler="gcc" />
				<Compiler>
					<Add option="-g" />
				</Compiler>
			</Target>
			<Target title="Release">
				<Option output="bin/Release/Lessons" prefix_auto="1" extension_auto="1" />
				<Option object_output="obj/Release/" />
				<Option type="1" />
				<Option compiler="gcc" />
				<Compiler>
					<Add option="-O2" />
				</Compiler>
				<Linker>
					<Add option="-s" />
				</Linker>
			</Target>
		</Build>
		<Compiler>
			<Add option="-Wall" />
		</Compiler>
		<Unit filename="Lesson1/Math1.cpp" />
		<Unit filename="Lesson1/Math2.cpp" />
		<Unit filename="Lesson2/Doors1.cpp" />
		<Unit filename="Lesson2/Fate.cpp" />
		<Unit filename="Lesson3/Right Triangle.cpp" />
		<Unit filename="Lesson3/countersomthingorother.cpp" />
		<Unit filename="Lesson3/stupidstuff.cpp" />
		<Extensions>
			<code_completion />
			<envvars />
			<debugger />
			<lib_finder disable_auto="1" />
		</Extensions>
	</Project>
</CodeBlocks_project_file>
