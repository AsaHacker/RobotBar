cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_EXE_LINKER_FLAGS "-llua -lluabind ofx/libs/openFrameworksCompiled/lib/osx/libopenFrameworks.a  -Fofx/libs/glut/lib/osx/  -mmacosx-version-min=10.7 -framework Accelerate -framework QTKit -framework GLUT -framework AGL -framework ApplicationServices -framework AudioToolbox -framework Carbon -framework CoreAudio -framework CoreFoundation -framework CoreServices -framework OpenGL -framework QuickTime -framework IOKit -framework Cocoa -framework CoreVideo  ofx/libs/FreeImage/lib/osx/freeimage.a  ofx/libs/assimp/lib/osx/assimp.a  ofx/libs/cairo/lib/osx/cairo-script-interpreter.a  ofx/libs/cairo/lib/osx/cairo.a  ofx/libs/cairo/lib/osx/pixman-1.a  ofx/libs/freetype/lib/osx/freetype.a ofx/libs/glew/lib/osx/glew.a  ofx/libs/glfw/lib/osx/libglfw3.a  ofx/libs/openssl/lib/osx/crypto.a  ofx/libs/openssl/lib/osx/ssl.a  ofx/libs/poco/lib/osx/PocoCrypto.a ofx/libs/poco/lib/osx/PocoData.a  ofx/libs/poco/lib/osx/PocoDataODBC.a  ofx/libs/poco/lib/osx/PocoDataSQLite.a ofx/libs/poco/lib/osx/PocoFoundation.a ofx/libs/poco/lib/osx/PocoNet.a  ofx/libs/poco/lib/osx/PocoNetSSL.a  ofx/libs/poco/lib/osx/PocoUtil.a  ofx/libs/poco/lib/osx/PocoXML.a  ofx/libs/poco/lib/osx/PocoZip.a  ofx/libs/rtAudio/lib/osx/rtAudio.a  ofx/libs/tess2/lib/osx/tess2.a  /System/Library/PrivateFrameworks/MultitouchSupport.framework/MultitouchSupport  ofx/libs/fmodex/lib/osx/libfmodex.dylib -lobjc")
project(ssb_proj_pkg) 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV)

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ssb_proj_pkg
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/Cellar/boost/1.55.0/include
  ofx/libs/openFrameworks
  ofx/libs/openFrameworks/3d
  ofx/libs/openFrameworks/app
  ofx/libs/openFrameworks/communication
  ofx/libs/openFrameworks/events
  ofx/libs/openFrameworks/gl
  ofx/libs/openFrameworks/graphics
  ofx/libs/openFrameworks/math
  ofx/libs/openFrameworks/sound
  ofx/libs/openFrameworks/types
  ofx/libs/openFrameworks/utils
  ofx/libs/openFrameworks/video
  ofx/libs/glew/include
  ofx/libs/tess2/include
  ofx/libs/poco/include
  ofx/libs/assimp/include
  ofx/libs/cairo/include
  ofx/libs/cairo/include/cairo
  ofx/libs/cairo/include/pixman-1/pixman
  ofx/libs/cairo/include/libpng15
  ofx/libs/fmodex/include
  ofx/libs/freetype/include
  ofx/addons/ofxCv/src
  ofx/addons/ofxCv/libs/ofxCv/include
  ofx/addons/ofxCv/libs/CLD/include/CLD
  ofx/addons/ofxFaceTracker/src
  ofx/addons/ofxFaceTracker/libs/FaceTracker/include
  ofx/addons/ofxLua/src
  ofx/addons/ofxLua/libs/lua
  ofx/addons/ofxLua/libs/lua/lua
  ofx/addons/ofxLua/libs/luabind
  ofx/addons/ofxLua/src/bindings
  ofx/addons/ofxTrueTypeFontUC/src
  ofx/libs/freetype/include/freetype2
  ofx/addons/ofxSTL/src
  ofx/addons/ofxSTL/src/utils
  ofx/addons/ofxMultiTouchPad/src
  ofx/addons/ofxUI/src
  ofx/addons/ofxXmlSettings/src
  src/common
  src/sensors
  src/actuators
  src/media
  src/utils
  src/ui_tools
)


## Declare a cpp executable
add_executable(app_test examples/app_test.cc src/common/ssb_common_vec.cc)

add_executable(free_sketch examples/free_sketch.cc src/common/ssb_common_vec.cc)

add_executable(demo_maker examples/demo_maker.cc src/common/ssb_common_vec.cc src/utils/ssb_utils_model.cc src/ui_tools/ssb_uitools_canvas.cc src/ui_tools/ssb_uitools_keyframe.cc src/utils/ssb_utils_interpolation.cc  ofx/addons/ofxUI/src/ofxUILabelButton.cpp  ofx/addons/ofxUI/src/ofxUIRectangle.cpp  ofx/addons/ofxUI/src/ofxUI2DGraph.cpp	 ofx/addons/ofxUI/src/ofxUIEventArgs.cpp  ofx/addons/ofxUI/src/ofxUITextInput.cpp  ofx/addons/ofxUI/src/ofxUILabelToggle.cpp  ofx/addons/ofxUI/src/ofxUIRotarySlider.cpp  ofx/addons/ofxUI/src/ofxUI2DPad.cpp  ofx/addons/ofxUI/src/ofxUIFPS.cpp  ofx/addons/ofxUI/src/ofxUIToggle.cpp  ofx/addons/ofxUI/src/ofxUIMinimalSlider.cpp  ofx/addons/ofxUI/src/ofxUIScrollableCanvas.cpp  ofx/addons/ofxUI/src/ofxUIBaseDraws.cpp  ofx/addons/ofxUI/src/ofxUIFPSSlider.cpp  ofx/addons/ofxUI/src/ofxUIToggleMatrix.cpp  ofx/addons/ofxUI/src/ofxUIMovingGraph.cpp  ofx/addons/ofxUI/src/ofxUISlider.cpp  ofx/addons/ofxUI/src/ofxUIBiLabelSlider.cpp	 ofx/addons/ofxUI/src/ofxUIImage.cpp  ofx/addons/ofxUI/src/ofxUIMultiImageButton.cpp  ofx/addons/ofxUI/src/ofxUISortableList.cpp  ofx/addons/ofxUI/src/ofxUIValuePlotter.cpp  ofx/addons/ofxUI/src/ofxUIButton.cpp  ofx/addons/ofxUI/src/ofxUIImageButton.cpp  ofx/addons/ofxUI/src/ofxUIMultiImageToggle.cpp  ofx/addons/ofxUI/src/ofxUISpacer.cpp  ofx/addons/ofxUI/src/ofxUIWaveform.cpp  ofx/addons/ofxUI/src/ofxUICanvas.cpp  ofx/addons/ofxUI/src/ofxUIImageSampler.cpp  ofx/addons/ofxUI/src/ofxUINumberDialer.cpp  ofx/addons/ofxUI/src/ofxUISpectrum.cpp  ofx/addons/ofxUI/src/ofxUIWidget.cpp  ofx/addons/ofxUI/src/ofxUICircleSlider.cpp	 ofx/addons/ofxUI/src/ofxUIImageSlider.cpp  ofx/addons/ofxUI/src/ofxUISuperCanvas.cpp  ofx/addons/ofxUI/src/ofxUIWidgetWithLabel.cpp  ofx/addons/ofxUI/src/ofxUIImageToggle.cpp	 ofx/addons/ofxUI/src/ofxUIRadio.cpp  ofx/addons/ofxUI/src/ofxUIDragableLabelButton.cpp	 ofx/addons/ofxUI/src/ofxUITabBar.cpp  ofx/addons/ofxUI/src/ofxUILabel.cpp  ofx/addons/ofxUI/src/ofxUIRangeSlider.cpp  ofx/addons/ofxUI/src/ofxUIDropDownList.cpp  ofx/addons/ofxUI/src/ofxUITextArea.cpp  ofx/addons/ofxXmlSettings/src/ofxXmlSettings.cpp ofx/addons/ofxXmlSettings/libs/tinyxml.cpp ofx/addons/ofxXmlSettings/libs/tinyxmlerror.cpp ofx/addons/ofxXmlSettings/libs/tinyxmlparser.cpp  ofx/addons/ofxTrueTypeFontUC/src/ofxTrueTypeFontUC.cpp ofx/addons/ofxSTL/src/ofxSTLImporter.cpp ofx/addons/ofxSTL/src/ofxSTLExporter.cpp)


target_link_libraries(app_test
  ${catkin_LIBRARIES}  
)

target_link_libraries(free_sketch
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
)

target_link_libraries(demo_maker
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

