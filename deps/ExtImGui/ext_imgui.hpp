#pragma once

#include <cassert>
#include <cctype>
#include <memory>

#include <functional>
#include <string>
#include <string_view>
#include <vector>

#ifdef EXT_IMGUI_DLL
#ifdef EXT_IMGUI_DLL_EXPORTS
#define EXT_IMGUI_EXPORTS __declspec(dllexport)
#else
#define EXT_IMGUI_EXPORTS __declspec(dllimport)
#endif
#else
#define EXT_IMGUI_EXPORTS
#endif

// From ImGui
#define ARRAYSIZE(_ARR) ((int)(sizeof(_ARR) / sizeof(*_ARR)))

namespace ExtImGui {
EXT_IMGUI_EXPORTS void Init(uint32_t resX, uint32_t resY);
EXT_IMGUI_EXPORTS void Shutdown();

EXT_IMGUI_EXPORTS bool Update();

EXT_IMGUI_EXPORTS class IObject *
RegisterObject(std::unique_ptr<class IObject> &&);

class IObject {
public:
	bool open = true;

  virtual void PreUpdate(){};
  virtual int Update() = 0;
  virtual void PostUpdate(){};

  virtual ~IObject() noexcept = default;
};

class OutputField : public IObject {
  std::vector<char *> m_items;
  bool m_scrollToBottom;

public:
  OutputField() = default;
  ~OutputField() noexcept override = default;

  EXT_IMGUI_EXPORTS int Update() override;

  template <typename... Args> void AddLog(const char *fmt, Args...);

  void ClearLog();

  void ScrollToBottom() { m_scrollToBottom = true; };
};

static char *Strdup(const char *str) {
  size_t len = strlen(str) + 1;
  void *buff = malloc(len);
  return (char *)memcpy(buff, (const void *)str, len);
}

template <typename... Args>
void OutputField::AddLog(const char *fmt, Args... args) {
  // FIXME-OPT
  char buf[1024];
  snprintf(buf, ARRAYSIZE(buf), fmt, args...);
  buf[ARRAYSIZE(buf) - 1] = 0;
  m_items.push_back(Strdup(buf));
  m_scrollToBottom = true;
}

class Console : public IObject {
  struct Command {
    std::string name;
    std::function<void(const std::vector<std::string> &)> function;
  };

  char m_inputBuffer[256];
  std::vector<char *> m_history;
  int m_historyPos; // -1: new line, 0..History.Size-1 browsing history.
  std::vector<Command> m_commands;

  OutputField *m_outputField;

  EXT_IMGUI_EXPORTS Console(OutputField *);

  int TextEditCallback(void *data);

public:
  template <typename Output> friend Console *CreateConsole();

  ~Console() noexcept override { delete m_outputField; };

  template <typename... Args> void AddLog(const char *fmt, Args... args) {
    m_outputField->AddLog(fmt, args...);
  };

  EXT_IMGUI_EXPORTS void ClearLog();

  void
  AddCommand(std::string name,
             std::function<void(const std::vector<std::string> &)> function) {
    m_commands.push_back({std::move(name), function});
  };
  EXT_IMGUI_EXPORTS void ExecCommand(std::string_view command_line);

  int Update() override;
};

template <typename Output = OutputField> Console *CreateConsole() {
  auto rawc = new Console(new Output());
  std::unique_ptr<Console> c(rawc);

  return static_cast<Console *>(RegisterObject(std::move(c)));
};

class Performances : public IObject {
public:
  int Update() override;
};

EXT_IMGUI_EXPORTS Performances *CreatePerformancesWidget();

} // namespace ExtImGui

#undef ARRAYSIZE
