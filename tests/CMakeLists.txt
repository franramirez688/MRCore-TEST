cmake_minimum_required (VERSION 2.6)

PROJECT(TEST)



		INCLUDE_DIRECTORIES(C:/TEMP/mrcore/include/mrcore) 
		LINK_DIRECTORIES(C:/TEMP/mrcore/lib)


	ADD_EXECUTABLE(TEST main.cpp)

	
	TARGET_LINK_LIBRARIES(TEST debug mrcore_d opengl32 glu32)	
	TARGET_LINK_LIBRARIES(TEST optimized mrcore opengl32 glu32)

