# Temperature compensation support
#
# Copyright (C) 2024  Your Name <your@email.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging, bisect

class TemperatureCompensation:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.enabled = config.getboolean('enabled', False)
        self.temp_table = []
        
        # 解析温度补偿表
        try:
            temp_comp_str = config.get('temperature_compensation')
            # 按行分割并解析每一行
            for line in temp_comp_str.split('\n'):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                try:
                    # 分割并转换为浮点数
                    display_temp, actual_temp = [float(x.strip()) for x in line.split(':')]
                    self.temp_table.append((display_temp, actual_temp))
                except Exception as e:
                    raise config.error(
                        "Invalid temperature compensation value '%s': %s" % (line, str(e)))
        except Exception as e:
            raise config.error(
                "Error loading temperature compensation table: %s" % str(e))
        
        # 按显示温度排序
        if self.temp_table:
            self.temp_table.sort()  # 按显示温度排序
            self.displayed_temps = [t[0] for t in self.temp_table]
            self.actual_temps = [t[1] for t in self.temp_table]
            logging.info("Loaded temperature compensation table for %s: %s",
                        self.name, self.temp_table)
        
        self.heater = None
        self.printer.register_event_handler("klippy:connect", 
                                          self._handle_connect)
        
        # 注册G-code命令 - 只在第一个温度补偿实例时注册
        gcode = self.printer.lookup_object('gcode')
        if not hasattr(gcode, '_temp_compensation_cmd_registered'):
            gcode.register_command('SET_TEMP_COMPENSATION',
                                 self.cmd_SET_TEMP_COMPENSATION,
                                 desc=self.cmd_SET_TEMP_COMPENSATION_help)
            # 标记命令已注册
            gcode._temp_compensation_cmd_registered = True
    
    def _handle_connect(self):
        # Connect to the corresponding heater
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.name)
        if self.heater is not None:
            self.heater.set_temp_comp(self)
            # Set initial enable state
            if self.enabled:
                self.heater.enable_temp_comp()
            else:
                self.heater.disable_temp_comp()
            logging.info("Connected temperature compensation to heater %s (enabled: %s)", 
                        self.name, self.enabled)
    
    def get_compensated_temp(self, display_temp):
        """将显示温度转换为实际温度"""
        if not self.temp_table:
            logging.info("No temp_table, returning original temp: %.1f", display_temp)
            return display_temp
        
        logging.info("Converting display_temp %.1f to actual temp using table: %s",
                    display_temp, self.temp_table)
        
        # 如果温度超出范围，使用最近的端点
        if display_temp <= self.displayed_temps[0]:
            logging.info("Display temp %.1f below range, using min actual temp: %.1f",
                        display_temp, self.actual_temps[0])
            return self.actual_temps[0]
        if display_temp >= self.displayed_temps[-1]:
            logging.info("Display temp %.1f above range, using max actual temp: %.1f",
                        display_temp, self.actual_temps[-1])
            return self.actual_temps[-1]
        
        # 使用线性插值计算实际温度
        idx = bisect.bisect_right(self.displayed_temps, display_temp) - 1
        d_temp1, a_temp1 = self.temp_table[idx]
        d_temp2, a_temp2 = self.temp_table[idx + 1]
        factor = (display_temp - d_temp1) / (d_temp2 - d_temp1)
        actual_temp = a_temp1 + factor * (a_temp2 - a_temp1)
        logging.info("Interpolated actual temp for display temp %.1f: %.1f (between %.1f and %.1f)",
                    display_temp, actual_temp, a_temp1, a_temp2)
        return actual_temp
    
    def get_display_temp(self, actual_temp):
        """Convert actual temperature to display temperature"""
        if not self.temp_table:
            return actual_temp
            
        try:
            # 温度低于50度时直接显示实际温度
            if actual_temp <= 50.0:
                return actual_temp
                
            # 如果只有一组数据,使用简单的比例关系
            if len(self.temp_table) == 1:
                if actual_temp <= self.actual_temps[0]:
                    # 50度到第一个点之间使用线性过渡
                    ratio = self.displayed_temps[0] / self.actual_temps[0]
                    factor = (actual_temp - 50.0) / (self.actual_temps[0] - 50.0)
                    display_temp = actual_temp * (1.0 + (ratio - 1.0) * factor)
                    logging.info("Single point transition: actual=%.1f, ratio=%.3f, factor=%.3f, display=%.1f",
                                actual_temp, ratio, factor, display_temp)
                    return display_temp
                else:
                    # 高于第一个点时使用固定比例
                    ratio = self.displayed_temps[0] / self.actual_temps[0]
                    display_temp = actual_temp * ratio
                    logging.info("Single point scaling: actual=%.1f, ratio=%.3f, display=%.1f",
                                actual_temp, ratio, display_temp)
                    return display_temp
            
            # 多组数据的情况
            # 如果温度在50度到第一个补偿点之间,使用线性过渡
            if actual_temp <= self.actual_temps[0]:
                # 使用第一组数据计算比例
                ratio = self.displayed_temps[0] / self.actual_temps[0]
                factor = (actual_temp - 50.0) / (self.actual_temps[0] - 50.0)
                display_temp = actual_temp * (1.0 + (ratio - 1.0) * factor)
                logging.info("Multi-point transition: actual=%.1f, ratio=%.3f, factor=%.3f, display=%.1f",
                            actual_temp, ratio, factor, display_temp)
                return display_temp
            
            # 在补偿范围内使用线性插值
            for i in range(len(self.actual_temps)):
                if actual_temp <= self.actual_temps[i]:
                    if i == 0:
                        return self.displayed_temps[0]
                    
                    # 线性插值计算显示温度
                    a_temp1 = self.actual_temps[i-1]    # 实际温度1
                    a_temp2 = self.actual_temps[i]      # 实际温度2
                    d_temp1 = self.displayed_temps[i-1] # 显示温度1
                    d_temp2 = self.displayed_temps[i]   # 显示温度2
                    
                    factor = (actual_temp - a_temp1) / (a_temp2 - a_temp1)
                    display_temp = d_temp1 + factor * (d_temp2 - d_temp1)
                    
                    logging.info("In range interpolation: actual=%.1f, d1=%.1f, d2=%.1f, a1=%.1f, a2=%.1f, factor=%.3f, display=%.1f",
                                actual_temp, d_temp1, d_temp2, a_temp1, a_temp2, factor, display_temp)
                    return display_temp
            
            # 如果温度高于补偿范围,使用最后一组数据的比例
            ratio = self.displayed_temps[-1] / self.actual_temps[-1]
            display_temp = actual_temp * ratio
            logging.info("Above range scaling: actual=%.1f, ratio=%.3f, display=%.1f",
                        actual_temp, ratio, display_temp)
            return display_temp
            
        except Exception as e:
            logging.exception("Error calculating display temp: %s", str(e))
            return actual_temp
    
    def get_status(self, eventtime):
        return {
            'temperature_compensation': self.temp_table,
            'enabled': self.enabled,
            'active': self.heater.is_temp_comp_enabled() if self.heater else False
        }
    
    cmd_SET_TEMP_COMPENSATION_help = "Enable/disable temperature compensation for specified heater"
    def cmd_SET_TEMP_COMPENSATION(self, gcmd):
        # 获取指定的加热器名称，如果没有指定则使用当前实例的加热器
        heater_name = gcmd.get('HEATER', self.name)
        
        # 获取所有温度补偿实例
        pheaters = self.printer.lookup_object('heaters')
        all_temp_comps = pheaters.get_all_temp_comps()
        
        # 查找指定加热器的温度补偿实例
        temp_comp = None
        for comp in all_temp_comps:
            if comp.name == heater_name:
                temp_comp = comp
                break
                
        if temp_comp is None:
            raise gcmd.error("No temperature compensation found for heater '%s'" % heater_name)
            
        if temp_comp.heater is None:
            raise gcmd.error("No heater connected for temperature compensation")
            
        # 设置启用状态
        value = gcmd.get_int('ENABLE', 1)
        if value:
            temp_comp.heater.enable_temp_comp()
        else:
            temp_comp.heater.disable_temp_comp()
            
        gcmd.respond_info("Temperature compensation for %s %s" % 
                         (heater_name, "enabled" if value else "disabled"))
    
    def get_control_temp(self, target_temp):
        """Convert target temperature to control temperature"""
        if not self.temp_table:
            return target_temp
        
        try:
            # 温度低于50度时直接返回目标温度
            if target_temp <= 50.0:
                return target_temp
                
            # 如果只有一组数据,使用简单的比例关系
            if len(self.temp_table) == 1:
                if target_temp <= self.displayed_temps[0]:
                    # 50度到第一个点之间使用线性过渡
                    ratio = self.actual_temps[0] / self.displayed_temps[0]
                    factor = (target_temp - 50.0) / (self.displayed_temps[0] - 50.0)
                    control_temp = target_temp * (1.0 + (ratio - 1.0) * factor)
                    logging.info("Single point transition: target=%.1f, ratio=%.3f, factor=%.3f, control=%.1f",
                                target_temp, ratio, factor, control_temp)
                    return control_temp
                else:
                    # 高于第一个点时使用固定比例
                    ratio = self.actual_temps[0] / self.displayed_temps[0]
                    control_temp = target_temp * ratio
                    logging.info("Single point scaling: target=%.1f, ratio=%.3f, control=%.1f",
                                target_temp, ratio, control_temp)
                    return control_temp
            
            # 多组数据的情况
            # 如果温度在50度到第一个补偿点之间,使用线性过渡
            if target_temp <= self.displayed_temps[0]:
                # 使用第一组数据计算比例
                ratio = self.actual_temps[0] / self.displayed_temps[0]
                factor = (target_temp - 50.0) / (self.displayed_temps[0] - 50.0)
                control_temp = target_temp * (1.0 + (ratio - 1.0) * factor)
                logging.info("Multi-point transition: target=%.1f, ratio=%.3f, factor=%.3f, control=%.1f",
                            target_temp, ratio, factor, control_temp)
                return control_temp
            
            # 在补偿范围内使用线性插值
            for i in range(len(self.displayed_temps)):
                if target_temp <= self.displayed_temps[i]:
                    if i == 0:
                        return self.actual_temps[0]
                    
                    # 线性插值计算控制温度
                    d_temp1 = self.displayed_temps[i-1] # 显示温度1
                    d_temp2 = self.displayed_temps[i]   # 显示温度2
                    a_temp1 = self.actual_temps[i-1]    # 实际温度1
                    a_temp2 = self.actual_temps[i]      # 实际温度2
                    
                    factor = (target_temp - d_temp1) / (d_temp2 - d_temp1)
                    control_temp = a_temp1 + factor * (a_temp2 - a_temp1)
                    
                    logging.info("In range interpolation: target=%.1f, d1=%.1f, d2=%.1f, a1=%.1f, a2=%.1f, factor=%.3f, control=%.1f",
                                target_temp, d_temp1, d_temp2, a_temp1, a_temp2, factor, control_temp)
                    return control_temp
            
            # 如果温度高于补偿范围,使用最后一组数据的比例
            ratio = self.actual_temps[-1] / self.displayed_temps[-1]
            control_temp = target_temp * ratio
            logging.info("Above range scaling: target=%.1f, ratio=%.3f, control=%.1f",
                        target_temp, ratio, control_temp)
            return control_temp
            
        except Exception as e:
            logging.exception("Error calculating control temp: %s", str(e))
            return target_temp
    
    def is_temp_comp_enabled(self):
        """Return whether temperature compensation is enabled"""
        return self.enabled and self.heater is not None

def load_config_prefix(config):
    return TemperatureCompensation(config) 