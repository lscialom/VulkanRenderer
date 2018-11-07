#include <vulkan/vulkan.hpp>

#include <iostream>

//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------

// Redundant pattern
#define CHECK_VK_RESULT_FATAL(vk_function, msg)                                \
  {                                                                            \
    vk::Result res;                                                            \
    if ((res = (vk::Result)vk_function) != vk::Result::eSuccess) {             \
      std::string err = "[FATAL]";                                             \
      err += msg;                                                              \
      err += " : ";                                                            \
      err += vk::to_string(res);                                               \
      throw std::runtime_error(err.c_str());                                   \
    }                                                                          \
  }

// For visibility's sake
#define TRY_CATCH_BLOCK(msg, code)                                             \
  try {                                                                        \
    code                                                                       \
  } catch (const std::exception &e) {                                          \
    std::cerr << e.what() << "\n"                                              \
              << "[FATAL]" << msg << std::endl;                                \
    abort();                                                                   \
  }

//-----------------------------------------------------------------------------
// DEBUG OUTPUT FORMATTING
//-----------------------------------------------------------------------------

#ifndef NDEBUG
static std::string
GetDebugMessagePrefix(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                      VkDebugUtilsMessageTypeFlagsEXT messageType) {
  std::string prefix = vk::to_string(
      static_cast<vk::DebugUtilsMessageSeverityFlagBitsEXT>(messageSeverity));

  prefix += " : ";
  prefix += vk::to_string(
      static_cast<vk::DebugUtilsMessageTypeFlagsEXT>(messageType));

  std::transform(prefix.begin(), prefix.end(), prefix.begin(), ::toupper);

  return prefix;
}

static std::string
FormatDebugMessage(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                   VkDebugUtilsMessageTypeFlagsEXT messageType,
                   const VkDebugUtilsMessengerCallbackDataEXT *callbackData) {
  char *buf = (char *)malloc(strlen(callbackData->pMessage) + 500);

  sprintf(buf, "%s - Message ID Number %d, Message ID Name : %s\n\t%s",
          GetDebugMessagePrefix(messageSeverity, messageType).c_str(),
          callbackData->messageIdNumber, callbackData->pMessageIdName,
          callbackData->pMessage);

  std::string message = buf;
  free(buf);

  return message;
}

static std::string StringifyDebugMessageObjects(
    const VkDebugUtilsMessengerCallbackDataEXT *callbackData) {
  if (callbackData->objectCount == 0)
    return std::string();

  char tmp[500];
  sprintf(tmp, "\n\n\t Objects - %d\n", callbackData->objectCount);

  std::string message(tmp);
  for (uint32_t object = 0; object < callbackData->objectCount; ++object) {
    char tmp_message[500];
    sprintf(
        tmp_message, "\t\t Object[%d] - Type %s, Value %p, Name \"%s\"\n",
        object,
        vk::to_string((vk::ObjectType)callbackData->pObjects[object].objectType)
            .c_str(),
        (void *)(callbackData->pObjects[object].objectHandle),
        callbackData->pObjects[object].pObjectName);

    message += tmp_message;
  }

  return message;
}

static std::string StringifyDebugMessageLabels(
    const VkDebugUtilsMessengerCallbackDataEXT *callbackData) {
  if (callbackData->cmdBufLabelCount == 0)
    return std::string();

  char tmp[500];
  sprintf(tmp, "\n\n\t Command Buffer Labels - %d\n",
          callbackData->cmdBufLabelCount);

  std::string message(tmp);
  for (uint32_t label = 0; label < callbackData->cmdBufLabelCount; ++label) {
    char tmp_message[500];
    sprintf(tmp_message, "\t\t Label[%d] - %s { %f, %f, %f, %f}\n", label,
            callbackData->pCmdBufLabels[label].pLabelName,
            callbackData->pCmdBufLabels[label].color[0],
            callbackData->pCmdBufLabels[label].color[1],
            callbackData->pCmdBufLabels[label].color[2],
            callbackData->pCmdBufLabels[label].color[3]);

    message += tmp_message;
  }

  return message;
}

static std::string GetFullFormattedDebugMessage(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT messageType,
    const VkDebugUtilsMessengerCallbackDataEXT *callbackData) {
  return FormatDebugMessage(messageSeverity, messageType, callbackData) +
         StringifyDebugMessageObjects(callbackData) +
         StringifyDebugMessageLabels(callbackData);
}
#endif
