/*
;    Project:       Open Vehicle Monitor System
;    Date:          14th March 2017
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011        Sonny Chen @ EPRO/DX
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include "ovms_log.h"
static const char *TAG = "config";

#include <unistd.h>
#include <sys/stat.h>
#include <string.h>
#include <sstream>
#include <dirent.h>
#include "ovms_config.h"
#include "ovms_command.h"
#include "ovms_events.h"

#define OVMS_CONFIGPATH "/store/ovms_config"
#define OVMS_MAXVALSIZE 2500
//#define OVMS_PERSIST_METADATA


OvmsConfig MyConfig __attribute__ ((init_priority (1400)));

void store_mount(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  MyConfig.mount();
  writer->puts("Mounted STORE");
  }

void store_unmount(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  MyConfig.unmount();
  writer->puts("Unmounted STORE");
  }

void config_list(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (!MyConfig.ismounted()) return;

  if (argc == 0)
    {
    // Show all parameters
    for (ConfigMap::iterator it=MyConfig.m_map.begin(); it!=MyConfig.m_map.end(); ++it)
      {
      writer->printf("%-20s %s\n", it->first.c_str(), it->second->GetTitle());
      }
    }
  else
    {
    // Show all instances for a particular parameter
    OvmsConfigParam *p = MyConfig.CachedParam(argv[0]);
    if (p)
      {
      writer->printf("%s (%s %s)\n",argv[0],
        (p->Readable()?"readable":"protected"),
        (p->Writable()?"writeable":"read-only"));
      for (ConfigParamMap::iterator it=p->m_map.begin(); it!=p->m_map.end(); ++it)
        {
        if (p->Readable())
          { writer->printf("  %s: %s\n",it->first.c_str(), it->second.c_str()); }
        else
          { writer->printf("  %s\n",it->first.c_str()); }
        }
      }
    }
  }

void config_set(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (!MyConfig.ismounted()) return;

  OvmsConfigParam *p = MyConfig.CachedParam(argv[0]);
  if (p==NULL)
    {
    writer->puts("Error: parameter not found");
    return;
    }

  if (!p->Writable())
    {
    writer->puts("Error: parameter is not writeable");
    return;
    }

  p->SetValue(argv[1],argv[2]);
  }

void config_rm(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (!MyConfig.ismounted()) return;

  OvmsConfigParam *p = MyConfig.CachedParam(argv[0]);
  if (p==NULL)
    {
    writer->puts("Error: parameter not found");
    return;
    }

  if (!p->Writable())
    {
    writer->puts("Error: parameter is not writeable");
    return;
    }

  if (p->DeleteInstance(argv[1]))
    return;
  if (strcmp(argv[1], "*") != 0)
    return;
  MyConfig.DeregisterParam(argv[0]);
  }

OvmsConfig::OvmsConfig()
  {
  ESP_LOGI(TAG, "Initialising CONFIG (1400)");

  OvmsCommand* cmd_store = MyCommandApp.RegisterCommand("store","STORE framework",NULL,"",0,0,true);
  cmd_store->RegisterCommand("mount","Mount STORE",store_mount,"",0,0,true);
  cmd_store->RegisterCommand("unmount","Unmount STORE",store_unmount,"",0,0,true);

  OvmsCommand* cmd_config = MyCommandApp.RegisterCommand("config","CONFIG framework",NULL,"",0,0,true);
  cmd_config->RegisterCommand("list","Show configuration parameters/instances",config_list,"[<param>]",0,1,true);
  cmd_config->RegisterCommand("set","Set parameter:instance=value",config_set,"<param> <instance> <value>",3,3,true);
  cmd_config->RegisterCommand("rm","Remove parameter:instance",config_rm,"<param> {<instance> | *}",2,2,true);

  RegisterParam("password", "Password store", true, false);
  }

OvmsConfig::~OvmsConfig()
  {
  }

esp_err_t OvmsConfig::mount()
  {
//  if (!spiffs_is_registered)
//    vfs_spiffs_register();

//  if (!spiffs_is_mounted)
//    spiffs_mount();

  if (m_mounted)
    return ESP_OK;

  memset(&m_store_fat,0,sizeof(esp_vfs_fat_sdmmc_mount_config_t));
  m_store_fat.format_if_mount_failed = true;
  m_store_fat.max_files = 5;
  esp_vfs_fat_spiflash_mount("/store", "store", &m_store_fat, &m_store_wlh);
  m_mounted = true;

  struct stat ds;
  if (stat(OVMS_CONFIGPATH, &ds) != 0)
    {
    puts("Initialising OVMS CONFIG within STORE");
    mkdir(OVMS_CONFIGPATH,0);
    }

  DIR *dir;
  struct dirent *dp;
  if ((dir = opendir(OVMS_CONFIGPATH)) == NULL)
    {
    ESP_LOGE(TAG, "Error: Cannot open config store directory");
    return ESP_ERR_NOT_FOUND;
    }
  while ((dp = readdir(dir)) != NULL)
    {
    // Register the param in case this was not already done
    if (CachedParam(dp->d_name) == NULL)
      RegisterParam(dp->d_name, "", true, false);
    }
  closedir(dir);

  for (ConfigMap::iterator it=MyConfig.m_map.begin(); it!=MyConfig.m_map.end(); ++it)
    {
    it->second->Load();
    }
  MyEvents.SignalEvent("config.mounted", NULL);
  return ESP_OK;
  }

esp_err_t OvmsConfig::unmount()
  {
//  if (spiffs_is_mounted)
//    spiffs_unmount(0);

  if (m_mounted)
    {
    esp_vfs_fat_spiflash_unmount("/store", m_store_wlh);
    m_mounted = false;
    MyEvents.SignalEvent("config.unmounted", NULL);
    }

  return ESP_OK;
  }

bool OvmsConfig::ismounted()
  {
  return m_mounted;
  }

void OvmsConfig::RegisterParam(std::string name, std::string title, bool writable, bool readable)
  {
  auto k = m_map.find(name);
  if (k == m_map.end())
    {
    OvmsConfigParam* p = new OvmsConfigParam(name, title, writable, readable);
    m_map[name] = p;
    }
  else
    {
    k->second->SetAccess(writable, readable);
    }
  }

void OvmsConfig::DeregisterParam(std::string name)
  {
  auto k = m_map.find(name);
  if (k != m_map.end())
    {
    k->second->DeleteParam();
    delete k->second;
    m_map.erase(k);
    }
  }

void OvmsConfig::SetParamValue(std::string param, std::string instance, std::string value)
  {
  OvmsConfigParam *p = CachedParam(param);
  if (p)
    {
    p->SetValue(instance,value);
    }
  }

void OvmsConfig::SetParamValueBinary(std::string param, std::string instance, std::string value)
  {
  size_t len = value.length();
  std::string hex;
  hex.reserve(len * 2);
  char buf[4];
  for (size_t i = 0; i < len; ++i)
    {
    unsigned char c = value.at(i);
    snprintf(buf, sizeof(buf), "%02X", c);
    hex += buf;
    }
  SetParamValue(param, instance, hex);
  }

void OvmsConfig::SetParamValueInt(std::string param, std::string instance, int value)
  {
  std::ostringstream ss;
  ss << value;
  SetParamValue(param, instance, std::string(ss.str()));
  }

void OvmsConfig::SetParamValueFloat(std::string param, std::string instance, float value)
  {
  std::ostringstream ss;
  ss << value;
  SetParamValue(param, instance, std::string(ss.str()));
  }

void OvmsConfig::SetParamValueBool(std::string param, std::string instance, bool value)
  {
  SetParamValue(param, instance, std::string(value ? "yes" : "no"));
  }

void OvmsConfig::DeleteInstance(std::string param, std::string instance)
  {
  OvmsConfigParam *p = CachedParam(param);
  if (p)
    {
    p->DeleteInstance(instance);
    }
  }

std::string OvmsConfig::GetParamValue(std::string param, std::string instance, std::string defvalue)
  {
  OvmsConfigParam *p = CachedParam(param);
  if (p && p->IsDefined(instance))
    {
    return p->GetValue(instance);
    }
  else
    {
    return defvalue;
    }
  }

std::string OvmsConfig::GetParamValueBinary(std::string param, std::string instance, std::string defvalue)
  {
  std::string hex = GetParamValue(param,instance);
  size_t len = hex.length();
  if (len == 0) return defvalue;
  if (hex.find_first_not_of("0123456789ABCDEF", 0) != std::string::npos || (len & 1))
    {
    ESP_LOGE(TAG, "Invalid non-hex value for config param %s instance %s",
      param.c_str(), instance.c_str());
    return defvalue;
    }
  std::string value;
  value.reserve(len / 2);
  char buf[4] = {0};
  for (size_t i = 0; i < len; i += 2)
    {
    buf[0] = hex.at(i);
    buf[1] = hex.at(i + 1);
    unsigned char c = strtoul(buf, NULL, 16);
    value += c;
    }
  return value;
  }

int OvmsConfig::GetParamValueInt(std::string param, std::string instance, int defvalue)
  {
  std::string value = GetParamValue(param,instance);
  if (value.length() == 0) return defvalue;
  return atoi(value.c_str());
  }

float OvmsConfig::GetParamValueFloat(std::string param, std::string instance, float defvalue)
  {
  std::string value = GetParamValue(param,instance);
  if (value.length() == 0) return defvalue;
  return atof(value.c_str());
  }

bool OvmsConfig::GetParamValueBool(std::string param, std::string instance, bool defvalue)
  {
  std::string value = GetParamValue(param,instance);
  if (value.length() == 0) return defvalue;
  if ((value == "yes")||(value == "1")||(value == "true"))
    return true;
  else
    return false;
  }

bool OvmsConfig::IsDefined(std::string param, std::string instance)
  {
  OvmsConfigParam *p = CachedParam(param);
  if (p == NULL) return false;
  return p->IsDefined(instance);
  }

OvmsConfigParam* OvmsConfig::CachedParam(std::string param)
  {
  if (!m_mounted) return NULL;

  auto k = m_map.find(param);
  if (k == m_map.end())
    return NULL;
  else
    return k->second;
  }

bool OvmsConfig::ProtectedPath(std::string path)
  {
#ifdef CONFIG_OVMS_DEV_CONFIGVFS
  return false;
#else
  return (path.find(OVMS_CONFIGPATH) != std::string::npos);
#endif // #ifdef CONFIG_OVMS_DEV_CONFIGVFS
  }

OvmsConfigParam::OvmsConfigParam(std::string name, std::string title, bool writable, bool readable)
  {
  m_name = name;
  m_title = title;
  m_writable = writable;
  m_readable = readable;
  m_loaded = false;

  if (MyConfig.ismounted())
    {
    LoadConfig();
    }
  }

OvmsConfigParam::~OvmsConfigParam()
  {
  }

void OvmsConfigParam::LoadConfig()
  {
  if (m_loaded) return;  // Protected against loading more than once

  std::string path(OVMS_CONFIGPATH);
  path.append("/");
  path.append(m_name);
  // ESP_LOGI(TAG, "Trying %s",path.c_str());
  FILE* f = fopen(path.c_str(), "r");
  if (f)
    {
    char* buf = new char[OVMS_MAXVALSIZE];
    while (fgets(buf, OVMS_MAXVALSIZE, f))
      {
      buf[strlen(buf)-1] = 0; // Remove trailing newline
#ifdef OVMS_PERSIST_METADATA
      // check for meta data:
      if (buf[0] == '#')
        {
        if (strncmp(buf, "#access=", 8) == 0)
          {
          m_readable = (strchr(buf+8, 'r') != NULL);
          m_writable = (strchr(buf+8, 'w') != NULL);
          continue;
          }
        else if (strncmp(buf, "#title=", 7) == 0)
          {
          m_title = buf+7;
          continue;
          }
        }
#endif // OVMS_PERSIST_METADATA
      // read instance:
      char *p = index(buf,char(9));
      if (p == NULL) p = index(buf,' ');
      if (p)
        {
        *p = 0; // Null terminate the key
        p++;    // and point to the value
        m_map[std::string(buf)] = std::string(p);
        // ESP_LOGI(TAG, "Loaded %s/%s=%s", m_name.c_str(), buf, p);
        }
      }
    delete[] buf;
    fclose(f);
    }
  m_loaded = true;
  }

void OvmsConfigParam::SetValue(std::string instance, std::string value)
  {
  if (m_map.find(instance) == m_map.end() || m_map[instance] != value)
    {
    m_map[instance] = value;
    RewriteConfig();
    MyEvents.SignalEvent("config.changed", this);
    }
  }

void OvmsConfigParam::DeleteParam()
  {
  std::string path(OVMS_CONFIGPATH);
  path.append("/");
  path.append(m_name);
  unlink(path.c_str());
  MyEvents.SignalEvent("config.changed", this);
  }

bool OvmsConfigParam::DeleteInstance(std::string instance)
  {
  bool ret = false;
  auto k = m_map.find(instance);
  if (k != m_map.end())
    {
    m_map.erase(k);
    RewriteConfig();
    ret = true;
    }
  MyEvents.SignalEvent("config.changed", this);
  return ret;
  }

std::string OvmsConfigParam::GetValue(std::string instance)
  {
  auto k = m_map.find(instance);
  if (k == m_map.end())
    return std::string("");
  else
    return k->second;
  }

bool OvmsConfigParam::IsDefined(std::string instance)
  {
  if (instance.empty())
    return !m_map.empty();
  auto k = m_map.find(instance);
  if (k == m_map.end())
    return false;
  else
    return true;
  }

bool OvmsConfigParam::Writable()
  {
  return m_writable;
  }

bool OvmsConfigParam::Readable()
  {
  return m_readable;
  }

void OvmsConfigParam::SetAccess(bool writable, bool readable)
  {
  m_writable = writable;
  m_readable = readable;
  }

std::string OvmsConfigParam::GetName()
  {
  return m_name;
  }

void OvmsConfigParam::RewriteConfig()
  {
  std::string path(OVMS_CONFIGPATH);
  path.append("/");
  path.append(m_name);
  FILE* f = fopen(path.c_str(), "w");
  if (f)
    {
#ifdef OVMS_PERSIST_METADATA
    // write meta data:
    fprintf(f, "#access=%s%s\n", m_readable ? "r" : "", m_writable ? "w" : "");
    fprintf(f, "#title=%s\n", m_title.c_str());
#endif
    // write instances:
    for (ConfigParamMap::iterator it=m_map.begin(); it!=m_map.end(); ++it)
      {
      fprintf(f,"%s\t%s\n",it->first.c_str(),it->second.c_str());
      }
    fclose(f);
    }
  }

void OvmsConfigParam::Load()
  {
  if (!m_loaded) LoadConfig();
  }
