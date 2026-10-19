/*
 * Screen 86/87 bridge.
 * This file only mirrors existing runtime alarm/fault tables into the
 * independent realtime event manager.  It never changes device state,
 * communication scheduling, linkage decisions, or flash history records.
 */
static uint32_t RealtimeEventMakeKey(uint8_t source, uint8_t loop,
                                     uint8_t address, uint8_t type,
                                     uint8_t detail)
{
    return ((uint32_t)(source & 0x0FU) << 28) |
           ((uint32_t)(loop & 0x0FU) << 24) |
           ((uint32_t)address << 16) |
           ((uint32_t)type << 8) | detail;
}

static void RealtimeEventCopyTime(RealtimeEventTime *target,
                                  const AlarmTimeRecord *source)
{
    target->year = source->years;
    target->month = source->months;
    target->day = source->days;
    target->hour = source->hours;
    target->minute = source->minute;
    target->second = source->second;
}

static void RealtimeEventCurrentTime(RealtimeEventTime *target)
{
    target->year = (uint16_t)years + 2000U;
    target->month = months;
    target->day = days;
    target->hour = hours;
    target->minute = minutes;
    target->second = secs;
}

static uint8_t RealtimePrinterDeviceType(uint8_t loop, uint8_t address)
{
    if(loop == 1U)
    {
        uint8_t type = getPointTypeMixtureDetectName(address);
        if(type == DEVICE_PRODUCT_XR8002_TEMP) return BSP_PRINTER_DEVICE_POINT_TEMP;
        if(type == DEVICE_PRODUCT_XR8001_SMOKE) return BSP_PRINTER_DEVICE_POINT_SMOKE;
        if(type == DEVICE_PRODUCT_FIM1017) return BSP_PRINTER_DEVICE_ISOLATOR;
        return BSP_PRINTER_DEVICE_COMPOUND;
    }
    if(loop == 2U)
    {
        switch(MBusCtrl_GetDeviceType(address))
        {
            case MBUS_CONTROL_DEV_SGBJQ:       return BSP_PRINTER_DEVICE_SOUND_LIGHT;
            case MBUS_CONTROL_DEV_XR2200:      return BSP_PRINTER_DEVICE_MANUAL_CALL;
            case MBUS_CONTROL_DEV_FIRE_DISPLAY:
            case MBUS_CONTROL_DEV_FIM1017:     return BSP_PRINTER_DEVICE_FIRE_DISPLAY;
            case MBUS_CONTROL_DEV_GCM1002:     return BSP_PRINTER_DEVICE_GAS_MODULE;
            case MBUS_CONTROL_DEV_FCM1011:     return BSP_PRINTER_DEVICE_INPUT_OUTPUT;
            case MBUS_CONTROL_DEV_FCM1012:     return BSP_PRINTER_DEVICE_DUAL_OUTPUT;
            case MBUS_CONTROL_DEV_FAN_BUTTON:  return BSP_PRINTER_DEVICE_FAN_BUTTON;
            case MBUS_CONTROL_DEV_SPRAY_BUTTON:return BSP_PRINTER_DEVICE_SPRAY_BUTTON;
            default:                           return BSP_PRINTER_DEVICE_UNKNOWN;
        }
    }
    if(loop == 3U) return BSP_PRINTER_DEVICE_COMPOUND;
    if(loop == 4U) return BSP_PRINTER_DEVICE_FAN_CONTROLLER;
    return BSP_PRINTER_DEVICE_UNKNOWN;
}

static uint8_t RealtimePrinterAlarmType(uint8_t alarm_type)
{
    switch(alarm_type)
    {
        case Temperature:              return BSP_PRINTER_EVENT_TEMP_ALARM;
        case Smoke:                    return BSP_PRINTER_EVENT_SMOKE_ALARM;
        case HandAlarm:                return BSP_PRINTER_EVENT_MANUAL_ALARM;
        case Carbon:
        case GasCarbonLowWarning:      return BSP_PRINTER_EVENT_CO_LOW;
        case GasCarbonHighWarning:     return BSP_PRINTER_EVENT_CO_HIGH;
        case Hydrogen:
        case GasHydrogenLowWarning:    return BSP_PRINTER_EVENT_H2_LOW;
        case GasHydrogenHighWarning:   return BSP_PRINTER_EVENT_H2_HIGH;
        default:                       return BSP_PRINTER_EVENT_NONE;
    }
}

static uint8_t RealtimePrinterFaultType(uint8_t loop, uint8_t fault_type)
{
    if(loop == 1U)
    {
        switch(fault_type)
        {
            case LOOP1_FAULT_OFFLINE:          return BSP_PRINTER_EVENT_COMM_FAULT;
            case LOOP1_FAULT_TEMPERATURE:      return BSP_PRINTER_EVENT_TEMP_FAULT;
            case LOOP1_FAULT_SMOKE_POLLUTION:
            case LOOP1_FAULT_SMOKE_SENSOR:     return BSP_PRINTER_EVENT_SMOKE_FAULT;
            case LOOP1_FAULT_FIM1017_BRANCH1:  return BSP_PRINTER_EVENT_BRANCH1_SHORT;
            case LOOP1_FAULT_FIM1017_BRANCH2:  return BSP_PRINTER_EVENT_BRANCH2_SHORT;
            case LOOP1_FAULT_FIM1017_BRANCH3:  return BSP_PRINTER_EVENT_BRANCH3_SHORT;
            default:                           return BSP_PRINTER_EVENT_DEVICE_FAULT;
        }
    }
    if(loop == 3U)
    {
        switch(fault_type)
        {
            case RS485_LOOP3_FAULT_OFFLINE:       return BSP_PRINTER_EVENT_COMM_FAULT;
            case RS485_LOOP3_FAULT_TEMPERATURE:   return BSP_PRINTER_EVENT_TEMP_FAULT;
            case RS485_LOOP3_FAULT_SMOKE:
            case RS485_LOOP3_FAULT_SMOKE_SENSOR:  return BSP_PRINTER_EVENT_SMOKE_FAULT;
            default:                              return BSP_PRINTER_EVENT_DEVICE_FAULT;
        }
    }
    if(loop == 4U)
    {
        if(fault_type == IG3302_FAULT_OFFLINE) return BSP_PRINTER_EVENT_COMM_FAULT;
        if(fault_type >= IG3302_FAULT_FAN1_PUSHROD &&
           fault_type <= IG3302_FAULT_FAN1_COMBINED)
            return BSP_PRINTER_EVENT_FAN1_FAULT;
        if(fault_type >= IG3302_FAULT_FAN2_PUSHROD &&
           fault_type <= IG3302_FAULT_FAN2_COMBINED)
            return BSP_PRINTER_EVENT_FAN2_FAULT;
    }
    if(loop == 2U) return BSP_PRINTER_EVENT_COMM_FAULT;
    return BSP_PRINTER_EVENT_DEVICE_FAULT;
}

static uint8_t RealtimePrinterHostFaultType(uint8_t package_id,
                                            uint8_t fault_state)
{
    switch(package_id)
    {
        case SYS_MAIN_POWER_KEY_ID:
            return BSP_PRINTER_HOST_MAIN_POWER;
        case SYS_BACK_POWER_KEY_ID:
            return (fault_state == SHORTCIRCUIT) ?
                   BSP_PRINTER_HOST_BACK_POWER_SHORT :
                   BSP_PRINTER_HOST_BACK_POWER_OPEN;
        case SYS_FLASH_FAULT_ID:
            return BSP_PRINTER_HOST_STORAGE;
        case SYS_RESET_COMM_FAULT_ID:
            return BSP_PRINTER_HOST_RESET_SEND;
        default:
            return BSP_PRINTER_HOST_FAULT_NONE;
    }
}

static void RealtimeEventObserveAndPrint(RealtimeEventKind kind, uint8_t category,
                                         uint32_t key, const RealtimeEventTime *time,
                                         uint8_t loop, uint8_t address,
                                         const uint8_t *event_name,
                                         uint8_t printer_event_type)
{
    if(BspRealtimeEvent_Observe(kind, category, key, time, loop, address,
                                event_name) != 0U &&
       loop >= 1U && loop <= 4U && printer_event_type != BSP_PRINTER_EVENT_NONE)
    {
        BspPrinterLoopEvent print_event;
        print_event.year = time->year;
        print_event.month = time->month;
        print_event.day = time->day;
        print_event.hour = time->hour;
        print_event.minute = time->minute;
        print_event.second = time->second;
        print_event.loop = loop;
        print_event.address = address;
        print_event.device_type = RealtimePrinterDeviceType(loop, address);
        print_event.event_type = printer_event_type;
        (void)BspPrinter_QueueLoopEvent(&print_event);
    }
}

static void RealtimeEventObserveAndPrintHostFault(uint8_t category,
                                                  uint32_t key,
                                                  const RealtimeEventTime *time,
                                                  uint8_t package_id,
                                                  uint8_t fault_state,
                                                  const uint8_t *event_name)
{
    uint8_t host_fault_type = RealtimePrinterHostFaultType(package_id, fault_state);
    if(BspRealtimeEvent_Observe(REALTIME_EVENT_FAULT, category, key, time,
                                0U, package_id, event_name) != 0U &&
       host_fault_type != BSP_PRINTER_HOST_FAULT_NONE)
    {
        BspPrinterHostFault print_fault;
        print_fault.year = time->year;
        print_fault.month = time->month;
        print_fault.day = time->day;
        print_fault.hour = time->hour;
        print_fault.minute = time->minute;
        print_fault.second = time->second;
        print_fault.fault_type = host_fault_type;
        (void)BspPrinter_QueueHostFault(&print_fault);
    }
}

static const char *RealtimeAlarmName(uint8_t alarm_type)
{
    switch(alarm_type)
    {
        case Temperature:
            return "\xCE\xC2\xB6\xC8\xBB\xF0\xBE\xAF";
        case Smoke:
            return "\xD1\xCC\xCE\xED\xBB\xF0\xBE\xAF";
        case HandAlarm:
            return "\xCA\xD6\xB6\xAF\xB1\xA8\xBE\xAF";
        case Carbon:
        case GasCarbonLowWarning:
            return "\xD2\xBB\xD1\xF5\xBB\xAF\xCC\xBC\xB5\xCD\xB1\xA8";
        case GasCarbonHighWarning:
            return "\xD2\xBB\xD1\xF5\xBB\xAF\xCC\xBC\xB8\xDF\xB1\xA8";
        case Hydrogen:
        case GasHydrogenLowWarning:
            return "H2\xB5\xCD\xB1\xA8";
        case GasHydrogenHighWarning:
            return "H2\xB8\xDF\xB1\xA8";
        default:
            return NULL;
    }
}

static uint8_t RealtimeAlarmLoopAddress(uint8_t detector_class,
                                        const DetectorAddrAttribute *address,
                                        uint8_t *loop, uint8_t *device)
{
    if(address->cluster_id == RS485_DETECT_FLASH_ID)
    {
        *loop = 3U;
        *device = address->pack_id;
        return 1U;
    }
    if(address->cluster_id == MBUS_CONTROL_FLASH_ID)
    {
        *loop = 2U;
        *device = address->pack_id;
        return 1U;
    }
    if(detector_class == CabinClassID)
    {
        *loop = 1U;
        *device = address->cabin_id;
        return 1U;
    }
    if(detector_class == PackClassID && address->pack_id != 0U)
    {
        *loop = 1U;
        *device = address->pack_id;
        return 1U;
    }
    return 0U;
}

static void RealtimeEventSyncAlarms(void)
{
    uint16_t index;
    RealtimeEventTime time;
    BspRealtimeEvent_BeginScan(REALTIME_EVENT_ALARM);

    for(index = 0U; index < pcfas.self_bottom_point; index++)
    {
        uint8_t loop;
        uint8_t address;
        const char *name = RealtimeAlarmName(pcfas.alarm_type[index]);
        if(name == NULL ||
           RealtimeAlarmLoopAddress(pcfas.detector_class[index],
                                    &pcfas.da[index], &loop, &address) == 0U)
            continue;
        RealtimeEventCopyTime(&time, &pcfas.atr[index]);
        RealtimeEventObserveAndPrint(REALTIME_EVENT_ALARM,
                                     REALTIME_ALARM_FIRE,
                                     RealtimeEventMakeKey(1U, loop, address,
                                                          pcfas.alarm_type[index], 0U),
                                     &time, loop, address,
                                     (const uint8_t *)name,
                                     RealtimePrinterAlarmType(pcfas.alarm_type[index]));
    }

    for(index = 0U; index < pcfws.self_bottom_point; index++)
    {
        uint8_t loop;
        uint8_t address;
        uint8_t alarm_type = pcfws.alarm_type[index];
        const char *name;
        if(alarm_type != Carbon && alarm_type != Hydrogen &&
           alarm_type != GasCarbonLowWarning &&
           alarm_type != GasCarbonHighWarning &&
           alarm_type != GasHydrogenLowWarning &&
           alarm_type != GasHydrogenHighWarning)
            continue;
        name = RealtimeAlarmName(alarm_type);
        if(name == NULL ||
           RealtimeAlarmLoopAddress(pcfws.detector_class[index],
                                    &pcfws.da[index], &loop, &address) == 0U)
            continue;
        RealtimeEventCopyTime(&time, &pcfws.atr[index]);
        RealtimeEventObserveAndPrint(REALTIME_EVENT_ALARM,
                                     REALTIME_ALARM_GAS,
                                     RealtimeEventMakeKey(2U, loop, address,
                                                          alarm_type, 0U),
                                     &time, loop, address,
                                     (const uint8_t *)name,
                                     RealtimePrinterAlarmType(alarm_type));
    }

    /* Loop-2 manual call points are not inserted into pcfas; mirror their live state. */
    for(index = 1U; index < MBUS_CONTROL_MAX_DEVICES; index++)
    {
        if(MBusCtrl_GetDeviceType((uint8_t)index) != MBUS_CONTROL_DEV_XR2200 ||
           MBusCtrl_GetOnline((uint8_t)index) == 0U ||
           MBusCtrl_IsDisconnected((uint8_t)index) != 0U ||
           MBusCtrl_IsAlarmState((uint8_t)index) == 0U)
            continue;
        RealtimeEventCurrentTime(&time);
        RealtimeEventObserveAndPrint(REALTIME_EVENT_ALARM,
                                     REALTIME_ALARM_FIRE,
                                     RealtimeEventMakeKey(3U, 2U, (uint8_t)index,
                                                          HandAlarm, 0U),
                                     &time, 2U, (uint8_t)index,
                                     (const uint8_t *)"\xCA\xD6\xB6\xAF\xB1\xA8\xBE\xAF",
                                     BSP_PRINTER_EVENT_MANUAL_ALARM);
    }

    /* Linkage and extinguishing categories are intentionally reserved only. */
    BspRealtimeEvent_EndScan(REALTIME_EVENT_ALARM);
}

static const char *RealtimeLoop1FaultName(uint8_t fault_type)
{
    switch(fault_type)
    {
        case LOOP1_FAULT_OFFLINE: return "\xB5\xF4\xCF\xDF";
        case LOOP1_FAULT_TEMPERATURE: return "\xCE\xC2\xB6\xC8\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF";
        case LOOP1_FAULT_SMOKE_POLLUTION: return "\xD1\xCC\xCE\xED\xCE\xDB\xC8\xBE\xB9\xCA\xD5\xCF";
        case LOOP1_FAULT_SMOKE_SENSOR: return "\xD1\xCC\xCE\xED\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF";
        case LOOP1_FAULT_FIM1017_BRANCH1: return "\xD6\xA7\xC2\xB7" "1" "\xB6\xCC\xC2\xB7";
        case LOOP1_FAULT_FIM1017_BRANCH2: return "\xD6\xA7\xC2\xB7" "2" "\xB6\xCC\xC2\xB7";
        case LOOP1_FAULT_FIM1017_BRANCH3: return "\xD6\xA7\xC2\xB7" "3" "\xB6\xCC\xC2\xB7";
        default: return "\xB9\xCA\xD5\xCF";
    }
}

static const char *RealtimeLoop3FaultName(uint8_t fault_type)
{
    switch(fault_type)
    {
        case RS485_LOOP3_FAULT_OFFLINE: return "\xB5\xF4\xCF\xDF";
        case RS485_LOOP3_FAULT_TEMPERATURE: return "\xCE\xC2\xB6\xC8\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF";
        case RS485_LOOP3_FAULT_SMOKE: return "\xD1\xCC\xCE\xED\xCE\xDB\xC8\xBE\xB9\xCA\xD5\xCF";
        case RS485_LOOP3_FAULT_CO: return "CO\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF";
        case RS485_LOOP3_FAULT_H2: return "H2\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF";
        case RS485_LOOP3_FAULT_VOC: return "VOC\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF";
        case RS485_LOOP3_FAULT_CH4: return "CH4\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF";
        case RS485_LOOP3_FAULT_SMOKE_SENSOR: return "\xD1\xCC\xCE\xED\xB4\xAB\xB8\xD0\xC6\xF7\xB9\xCA\xD5\xCF";
        default: return "\xB9\xCA\xD5\xCF";
    }
}

static const char *RealtimeControllerFaultName(uint8_t package_id)
{
    switch(package_id)
    {
        case SYS_FLASH_FAULT_ID: return "\xCF\xB5\xCD\xB3\xB4\xE6\xB4\xA2\xB9\xCA\xD5\xCF";
        case SYS_MAIN_POWER_KEY_ID: return "\xD6\xF7\xB5\xE7\xB9\xCA\xD5\xCF";
        case SYS_BACK_POWER_KEY_ID: return "\xB1\xB8\xB5\xE7\xB9\xCA\xD5\xCF";
        case SYS_RESET_COMM_FAULT_ID: return "\xB8\xB4\xCE\xBB\xB7\xA2\xCB\xCD\xD2\xEC\xB3\xA3";
        default: return "\xBF\xD8\xD6\xC6\xC6\xF7\xB9\xCA\xD5\xCF";
    }
}

static uint8_t RealtimeClassifyFault(const PackCabinFaultStorage *fault,
                                    uint8_t *loop, uint8_t *address,
                                    const char **name)
{
    if(fault->detector_class == CabinClassID)
    {
        *loop = 1U;
        *address = fault->da.cabin_id;
        *name = RealtimeLoop1FaultName(fault->fault_type);
        return REALTIME_FAULT_DETECTOR;
    }
    if(fault->detector_class == PackClassID)
    {
        *address = fault->da.pack_id;
        if(fault->da.cluster_id == RS485_DETECT_FLASH_ID)
        {
            *loop = 3U;
            *name = RealtimeLoop3FaultName(fault->fault_type);
            return REALTIME_FAULT_DETECTOR;
        }
        if(fault->da.cluster_id == MBUS_CONTROL_FLASH_ID)
        {
            uint8_t type = MBusCtrl_GetDeviceType(*address);
            *loop = 2U;
            *name = "\xB5\xF4\xCF\xDF";
            if(type == MBUS_CONTROL_DEV_FCM1011 || type == MBUS_CONTROL_DEV_FCM1012)
                return REALTIME_FAULT_EXTINGUISH;
            return REALTIME_FAULT_LINKAGE;
        }
        if(fault->da.cluster_id == IG3302_FLASH_ID)
        {
            *loop = 4U;
            *name = IG3302FaultName(fault->fault_type);
            return REALTIME_FAULT_LINKAGE;
        }
        *loop = 1U;
        *name = "\xB5\xF4\xCF\xDF";
        return REALTIME_FAULT_DETECTOR;
    }

    *loop = 0U;
    *address = fault->da.pack_id;
    if(fault->da.pack_id == OutFir1_Package_ID ||
       fault->da.pack_id == OutFir2_Package_ID ||
       fault->da.pack_id == CabinBK_Package_ID ||
       fault->da.pack_id == FEEDBK1_Package_ID ||
       fault->da.pack_id == FEEDBK2_Package_ID)
    {
        *name = "\xC3\xF0\xBB\xF0\xD7\xB0\xD6\xC3\xB9\xCA\xD5\xCF";
        return REALTIME_FAULT_EXTINGUISH;
    }
    if(fault->da.pack_id == SoundLt_Package_ID ||
       fault->da.pack_id == SirenBk_Package_ID ||
       fault->da.pack_id == Deflate_Package_ID ||
       fault->da.pack_id == HANDPOT_Package_ID)
    {
        *name = "\xC1\xAA\xB6\xAF\xC4\xA3\xBF\xE9\xB9\xCA\xD5\xCF";
        return REALTIME_FAULT_LINKAGE;
    }
    *name = RealtimeControllerFaultName(fault->da.pack_id);
    return REALTIME_FAULT_CONTROLLER;
}

static void RealtimeEventSyncFaults(void)
{
    uint16_t index;
    RealtimeEventTime time;
    BspRealtimeEvent_BeginScan(REALTIME_EVENT_FAULT);
    for(index = 0U; index < pcfs_buttom_point; index++)
    {
        uint8_t loop;
        uint8_t address;
        const char *name;
        uint8_t category = RealtimeClassifyFault(&pcfs[index], &loop,
                                                 &address, &name);
        RealtimeEventCopyTime(&time, &pcfs[index].atr);
        if(loop == 0U)
        {
            RealtimeEventObserveAndPrintHostFault(category,
                RealtimeEventMakeKey(4U, loop, address,
                                     pcfs[index].fault_type,
                                     pcfs[index].detector_class),
                &time, address, pcfs[index].da.cabin_id,
                (const uint8_t *)name);
        }
        else
        {
            RealtimeEventObserveAndPrint(REALTIME_EVENT_FAULT, category,
                                         RealtimeEventMakeKey(4U, loop, address,
                                                              pcfs[index].fault_type,
                                                              pcfs[index].detector_class),
                                         &time, loop, address,
                                         (const uint8_t *)name,
                                         RealtimePrinterFaultType(loop, pcfs[index].fault_type));
        }
    }

    for(index = 0U; index < DeviceRegistry_GetProductUnknownCount(); index++)
    {
        uint8_t loop = 0U;
        uint8_t address = 0U;
        uint8_t category;
        DeviceIdentifyError error = DEVICE_IDENTIFY_OK;
        if(DeviceRegistry_GetIdentifyErrorAt(index, &loop, &address, &error) == 0U)
            continue;
        category = (loop == 1U || loop == 3U) ? REALTIME_FAULT_DETECTOR :
                   ((loop == 2U || loop == 4U) ? REALTIME_FAULT_LINKAGE :
                    REALTIME_FAULT_CONTROLLER);
        RealtimeEventCurrentTime(&time);
        RealtimeEventObserveAndPrint(REALTIME_EVENT_FAULT, category,
                                     RealtimeEventMakeKey(5U, loop, address,
                                                          (uint8_t)error, 0U),
                                     &time, loop, address,
                                     (const uint8_t *)DeviceRegistry_GetIdentifyErrorText(error),
                                     BSP_PRINTER_EVENT_IDENTIFY_FAULT);
    }
    BspRealtimeEvent_EndScan(REALTIME_EVENT_FAULT);
}

static void RealtimeEventSyncAll(void)
{
    RealtimeEventSyncAlarms();
    RealtimeEventSyncFaults();
}
