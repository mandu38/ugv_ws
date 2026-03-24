import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chotaehyun/ugv_ws/install/ugv_chat_ai'
