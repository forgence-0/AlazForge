# alazforge_sample_integration — AlazForgeContext'i AlazEngine'in yapacağı
# gibi kullanan minimal, gerçekten çalışan konsol demosu.
add_executable(alazforge_sample_integration SampleIntegrationApp.cpp)
target_link_libraries(alazforge_sample_integration PRIVATE context)
