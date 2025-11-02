# TODO: use tests in test_msp as basis for functions to implement API class in msp_api.py with high level functions that do the job of parsing the raw data from the MSP packet, or converting the user's data into the right format for MSP; and demonstrate them here. API must only ever use the same serial socket object. 
# IMPORTANT:
# * No x.get("var",0), if value is invalid or missing, it's a bug and must except, do not substitute
# * in final payload presentation, always use same key names as payload if applicable
# * in final payload presentation, convert integerized unit (ie: cm, deci-degrees) into float customary units (ie: m, degrees)
# * do not "try except" enum gets, if value is missing it's a bug and must except
# * return raw enum type, not pure value/str: vbat_source = InavEnums.batVoltageSource_e(rep.get("vbatSource"))
# When working, run main.py or test_msp.py as needed, everything is already connected.