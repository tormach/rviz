string key # key of the parameter to set

uint8 STRING_VALUE = 0
uint8 FLOAT_VALUE = 1
uint8 BOOL_VALUE = 2
uint8 value_type # type of the parameter value

# parameter value, set one of these appropriately for value_type
string string_value
float32 float_value
bool bool_value
