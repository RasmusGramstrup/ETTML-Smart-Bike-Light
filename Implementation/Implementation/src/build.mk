## Standard behavior must be included here
INCLUDE_DIRS += $(SOURCE_PATH)/$(USRSRC) # add user sources to include path
CPPSRC += $(call target_files,$(USRSRC_SLASH),*.cpp)
CSRC += $(call target_files,$(USRSRC_SLASH),*.c)

APPSOURCES=$(call target_files,$(USRSRC_SLASH),*.cpp)
APPSOURCES+=$(call target_files,$(USRSRC_SLASH),*.c)

INCLUDE_DIRS += C:/Users/rgram/AppData/Roaming/Python/Python312/site-packages/emlearn
