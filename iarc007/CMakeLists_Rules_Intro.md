ＣＭakeＬists.txt 入门介绍官网：
http://wiki.ros.org/catkin/CMakeLists.txt

Overview（概览）

The file CMakeLists.txt is the input to the CMake build system for building software packages. Any CMake-compliant package contains one or more CMakeLists.txt file that describe how to build the code and where to install it to. The CMakeLists.txt file used for a catkin project is a standard vanilla CMakeLists.txt file with a few additional constraints.

通过设置ＣＭakeLists.txt操作ＣＭake编译系统去编译软件包和安装软件包。

Overall Structure and Ordering（总体结构）

Your CMakeLists.txt file MUST follow this format otherwise your packages will not build correctly. The order in the configuration DOES count.

    Required CMake Version (cmake_minimum_required) --- ＣＭake版本

    Package Name (project()) --- 包名

    Find other CMake/Catkin packages needed for build (find_package())  --- 添加编译过程需要的其他包

    Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())

    ------ 生成消息/服务/？？/ 
    Invoke message/service/action generation (generate_messages())
    ---- 引用依赖项

    Specify package build info export (catkin_package())
    ----- 加载相关包到环境变量中 

    Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
    ----- 生成库/可执行文件/？？

    Tests to build (catkin_add_gtest())
    ----- 编译测试程序

    Install rules (install()) 
    ----- 安装规则

CMake Version

Every catkin CMakeLists.txt file must start with the required version of CMake needed. Catkin requires version 2.8.3 or higher.

cmake_minimum_required(VERSION 2.8.3)

Package name

The next item is the name of the package which is specified by the CMake project function. Let us say we are making a package called robot_brain.

project(robot_brain)

Note in CMake you can reference the project name anywhere later in the CMake script by using the variable ${PROJECT_NAME} wherever needed.

${PROJECT_NAME} --- 现在的工程目录

Finding Dependent CMake Packages

We need to then specify which other CMake packages that need to be found to build our project using the CMake find_package function. There is always at least one dependency on catkin:

find_package(catkin REQUIRED)

------搜索编译依赖的工程

If your project depends on other wet packages, they are automatically turned into components (in terms of CMake) of catkin. Instead of using find_package on those packages, if you specify them as components, it will make life easier. For example, if you use the package nodelet.

find_package(catkin REQUIRED COMPONENTS nodelet)

           NB: You should only find_package components for which you want build flags. You should not add runtime dependencies.

You could also do:

find_package(catkin REQUIRED)
find_package(nodelet REQUIRED)

However, you will will see that this is an inconvenient way of doing things.

What Does find_package() Do?

----- CMake create enviroment variables about the found package

If a package is found by CMake through find_package, it results in the creation of several CMake environment variables that give information about the found package. These environment variables can be utilized later in the CMake script. The environment variables describe where the packages exported header files are, where source files are, what libraries the package depends on, and the paths of those libraries. The names always follow the convention of <PACKAGE NAME>_<PROPERTY>:

    <NAME>_FOUND - Set to true if the library is found, otherwise fall

    <NAME>_INCLUDE_DIRS or <NAME>_INCLUDES - The include paths exported by the package

    <NAME>_LIBRARIES or <NAME>_LIBS - The libraries exported by the package

    <NAME>_DEFINITIONS - ? 

Why Are Catkin Packages Specified as Components?

Catkin packages are not really components of catkin. Rather the components feature of CMake was utilized in the design of catkin to save you significant typing time.

For catkin packages, if you find_package them as components of catkin, this is advantageous as a single set of environment variables is created with the catkin_ prefix. For example, let us say you were using the package nodelet in your code. The recommended way of finding the package is:

find_package(catkin REQUIRED COMPONENTS nodelet)

This means that the include paths, libraries, etc exported by nodelet are also appended to the catkin_ variables. For example, catkin_INCLUDE_DIRS contains the include paths not only for catkin but also for nodelet as well! This will come in handy later.

We could alternatively find_package nodelet on its own:

find_package(nodelet)

This means the nodelet paths, libraries and so on would not be added to catkin_ variables.

This results in nodelet_INCLUDE_DIRS, nodelet_LIBRARIES, and so on. The same variables are also created using

find_package(catkin REQUIRED COMPONENTS nodelet)

Boost

If using C++ and Boost, you need to invoke find_package() on Boost and specify which aspects of Boost you are using as components. For example, if you wanted to use Boost threads, you would say:

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

catkin_package() is a catkin-provided CMake macro. This is required to specify catkin-specific information to the build system which in turn is used to generate pkg-config and CMake files.

------ 用来设定catkin信息

This function must be called before declaring any targets with add_library() or add_executable(). The function has 5 optional arguments:

    INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package

    LIBRARIES - The exported libraries from the project

    CATKIN_DEPENDS - Other catkin projects that this project depends on

    DEPENDS - Non-catkin CMake projects that this project depends on

    CFG_EXTRAS - Additional configuration options 

Full macro documentation can be found here.

As an example:

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp nodelet
   DEPENDS eigen opencv)

This indicates that the folder "include" within the package folder is where exported headers go. The CMake environment variable ${PROJECT_NAME} evaluates to whatever you passed to the project() function earlier, in this case it will be "robot_brain". "roscpp" + "nodelet" are packages that need to be present to build/run this package, and "eigen" + "opencv" are system dependencies that need to be present to build/run this package.

Specifying Build Targets

Build targets can take many forms, but usually they represent one of two possibilties:

    Executable Target - programs we can run
    Library Target - libraries that can be used by executable targets at build and/or runtime 

Target Naming

It is very important to note that the names of build targets in catkin must be unique regardless of the folders they are built/installed to. This is a requirement of CMake. However, unique names of targets are only necessary internally to CMake. One can have a target renamed to something else using the set_target_properties() function:

Example:

set_target_properties(rviz_image_view
                      PROPERTIES OUTPUT_NAME image_view
                      PREFIX "")

This will change the name of the target rviz_image_view to image_view in the build and install outputs.

Custom output directory

While the default output directory for executables and libraries is usual set to a reasonable value it must be customized in certain cases. I.e. a library containing Python bindings must be placed in a different folder to be importable in Python:

Example:

set_target_properties(python_module_library
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

Include Paths and Library Paths

Prior to specifying targets, you need to specify where resources can be found for said targets, specifically header files and libraries:

    Include Paths - Where can header files be found for the code (most common in C/C++) being built
    Library Paths - Where are libraries located that executable target build against?

    include_directories(<dir1>, <dir2>, ..., <dirN>)

    link_directories(<dir1>, <dir2>, ..., <dirN>) 

------ 库目录/包含目录

include_directories()

The argument to include_directories should be the *_INCLUDE_DIRS variables generated by your find_package calls and any additional directories that need to be included. If you are using catkin and Boost, your include_directories() call should look like:

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

The first argument "include" indicates that the include/ directory within the package is also part of the path.

------- 注意：库目录通常不需要添加，catkin/cmake在find_package（）中已经设定了链接信息，可以通过target_link_libraries()实现
link_directories()

The CMake link_directories() function can be used to add additional library paths, however, this is not recommended. //All catkin and CMake packages automatically have their link information added when they are find_packaged.// Simply link against the libraries in target_link_libraries()

Example:

link_directories(~/my_libs)

Please see this cmake thread to see a detailed example of using target_link_libraries() over link_directories().

Executable Targets

----- 生成可执行文件

To specify an executable target that must be built, we must use the add_executable() CMake function.

add_executable(myProgram src/main.cpp src/some_file.cpp src/another_file.cpp)

This will build a target executable called myProgram which is built from 3 source files: src/main.cpp, src/some_file.cpp and src/another_file.cpp.

Library Targets
------ 生成库文件

The add_library() CMake function is used to specify libraries to build. By default catkin builds shared libraries.

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

target_link_libraries

------- 设定可执行文件链接的库

Use the target_link_libraries() function to specify which libraries an executable target links against. This is done typically after an add_executable() call. Add ${catkin_LIBRARIES} if ros is not found.

Syntax:

target_link_libraries(<executableTargetName>, <lib1>, <lib2>, ... <libN>)

Example:

add_executable(foo src/foo.cpp)
add_library(moo src/moo.cpp)
target_link_libraries(foo moo)  -- This links foo against libmoo.so

Note that there is no need to use link_directories() in most use cases as that information is automatically pulled in via find_package().

Messages, Services, and Action Targets

---- 消息/服务/？？？在编译之前需要进行预处理步骤，以翻译为可编程语言为用户所用
Messages (.msg), services (.srv), and actions (.action) files in ROS require a special preprocessor build step before being built and used by ROS packages. The point of these macros is to generate programming language-specific files so that one can utilize messages, services, and actions in their programming language of choice. The build system will generate bindings using all available generators (e.g. gencpp, genpy, genlisp, etc).

There are three macros provided to handle messages, services, and actions respectively:

    add_message_files

    add_service_files

    add_action_files 

----- 生成消息/服务/？？？
These macros must then be followed by a call to the macro that invokes generation:

 generate_messages()

----- 调用生成器

Important Prerequisites/Constraints

    These macros must come BEFORE the catkin_package() macro in order for generation to work correctly. 
----- 这些宏必须在catkin_package（）之前引用

 find_package(catkin REQUIRED COMPONENTS ...)
 add_message_files(...)
 add_service_files(...)
 add_action_files(...)
 generate_messages(...)
 catkin_package(...)
 ...

    Your catkin_package() macro must have a CATKIN_DEPENDS dependency on message_runtime. 
-----catkin——package() 必须引用catkin依赖实时消息（ＣＡＴＫＩＮ_ＤＥＰＥＮＤＳ message_runtime）
catkin_package(
 ...
 CATKIN_DEPENDS message_runtime ...
 ...)

    You must use find_package() for the package message_generation, either alone or as a component of catkin: 

find_package(catkin REQUIRED COMPONENTS message_generation)
----- 必须调用消息生成包（message_generation）

    Your package.xml file must contain a build dependency on message_generation and a runtime dependency on message_runtime. This is not necessary if the dependencies are pulled in transitively from other packages.
------package.xml中包含 build 依赖项 message_generation
                        runtime 依赖项 message_runtime
------？？？如果从其他包传递进入则不必要？？？

    If you have a package which builds messages and/or services as well as executables that use them, you need to create an explicit dependency on the automatically-generated message target so that they are built in the correct order. (some_target is the name of the target set by add_executable()): 

----- 添加依赖项（可执行文件使用消息和服务）

  add_dependencies(some_target ${${PROJECT_NAME}_EXPORTED_TARGETS})

Example

If your package has two messages in a directory called "msg" named "MyMessage1.msg" and "MyMessage2.msg" and these messages depend on std_msgs and sensor_msgs and a service in a directory called "srv" named "MyService.srv" then you will need the following in your CMakeLists.txt:

切换行号显示

   1   # Get the information about this package's buildtime dependencies
   2   find_package(catkin REQUIRED
   3     COMPONENTS message_generation std_msgs sensor_msgs)
   4 
   5   # Declare the message files to be built
   6   add_message_files(FILES
   7     MyMessage1.msg
   8     MyMessage2.msg
   9   )
  10 
  11   # Declare the service files to be built
  12   add_service_files(FILES
  13     MyService.srv
  14   )
  15 
  16   # Actually generate the language-specific message and service files
  17   generate_messages(DEPENDENCIES std_msgs sensor_msgs)
  18 
  19   # Declare that this catkin package's runtime dependencies
  20   catkin_package(
  21    CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
  22   )

If, additionally, you want to build actionlib actions, and have an action specification file called "MyAction.action" in the "action" directory, you must add actionlib_msgs to the list of components which are find_packaged with catkin and add the following call before the call to generate_messages(...):

add_action_files(FILES
  MyAction.action
)

Furthermore the package must have a build dependency on actionlib_msgs.

Unit Tests

---- 测试
There is a catkin-specific macro for handling gtest-based unit tests called catkin_add_gtest().

catkin_add_gtest(myUnitTest test/utest.cpp)

Optional Step: Specifying Installable Targets

After build time, targets are placed into the devel space of the catkin workspace. However, often we want to install targets to the system (information about installation paths can be found in REP 122) so that they can be used by others or to a local folder to test a system-level installation. In other words, if you want to be able to do a "make install" of your code, you need to specify where targets should end up.

This is done using the CMake install() function which takes as arguments:

    TARGETS - which targets to install

    ARCHIVE DESTINATION - Static libraries and DLL (Windows) .lib stubs

    LIBRARY DESTINATION - Non-DLL shared libraries and modules

    RUNTIME DESTINATION - Executable targets and DLL (Windows) style shared libraries 

Take as an example:

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

Besides these standard destination some files must be installed to special folders. I.e. a library containing Python bindings must be installed to a different folder to be importable in Python:

install(TARGETS python_module_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

Installing Python Executable Scripts

For Python code, the install rule looks different as there is no use of the add_library() and add_executable() functions so as for CMake to determine which files are targets and what type of targets they are. Instead, use the following in your CMakeLists.txt file:

catkin_install_python(PROGRAMS scripts/myscript
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

Detailed information about installing python scripts and modules, as well as best practices for folder layout can be found in the catkin manual.
Installing header files

Header files must also be installed to the "include" folder, This is often done by installing the files of an entire folder (optionally filtered by filename patterns and excluding SVN subfolders). This can be done with an install rule that looks as follows:

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

or if the subfolder under include does not match the package name:

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

Installing roslaunch Files or Other Resources

Other resources like launchfiles can be installed to ${CATKIN_PACKAGE_SHARE_DESTINATION:

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
