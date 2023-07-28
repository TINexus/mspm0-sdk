Search.setIndex({"docnames": ["doc_guide-srcs/Stepper_Motor_Control_Overview", "doc_guide-srcs/drv8411a/DRV8411A_GUI_User_Guide", "doc_guide-srcs/drv8411a/DRV8411A_Hardware_User_Guide", "doc_guide-srcs/drv8889-q1/DRV8889-Q1_GUI_User_Guide", "doc_guide-srcs/drv8889-q1/DRV8889-Q1_Hardware_User_Guide", "doc_guide-srcs/index"], "filenames": ["doc_guide-srcs/Stepper_Motor_Control_Overview.rst", "doc_guide-srcs/drv8411a/DRV8411A_GUI_User_Guide.rst", "doc_guide-srcs/drv8411a/DRV8411A_Hardware_User_Guide.rst", "doc_guide-srcs/drv8889-q1/DRV8889-Q1_GUI_User_Guide.rst", "doc_guide-srcs/drv8889-q1/DRV8889-Q1_Hardware_User_Guide.rst", "doc_guide-srcs/index.rst"], "titles": ["Stepper Motor Control Library Overview", "DRV8411A GUI User Guide", "DRV8411A Hardware User Guide", "DRV8889-Q1 GUI User Guide", "DRV8889-Q1 Hardware User Guide", "Stepper Motor Control User Guide"], "terms": {"i": [0, 1, 2, 3, 4], "made": 0, "three": [0, 3], "main": 0, "thei": 0, "ar": [0, 1, 2, 3, 4], "architectur": 0, "The": [0, 1, 2, 3, 4], "user": 0, "specif": [0, 1, 3], "like": [0, 1, 3], "gui": [0, 2, 4, 5], "etc": 0, "present": 0, "thi": [0, 1, 3], "from": [0, 2, 3, 4], "variou": [0, 1, 3], "instanc": 0, "can": [0, 1, 2, 3, 4], "configur": [0, 1, 3], "us": [0, 2, 4, 5], "If": [0, 1, 2, 3, 4], "need": [0, 1, 3], "perform": [0, 1, 3], "ani": [0, 3], "hardwar": [0, 1, 3, 5], "action": [0, 1, 3], "recommend": 0, "abstract": 0, "creat": 0, "an": [0, 3], "provid": 0, "differ": 0, "peripher": 0, "goal": 0, "all": [0, 2, 3, 4], "which": [0, 1, 3], "simplifi": 0, "port": 0, "minim": 0, "updat": [0, 3], "other": [0, 1, 3], "compon": 0, "meant": 0, "onli": [0, 1, 3], "requir": [0, 1, 3, 5], "done": 0, "make": [0, 1, 3], "light": 0, "weight": 0, "while": [0, 3], "still": 0, "have": 0, "flexibl": 0, "design": 0, "number": [0, 1, 3], "channel": 0, "For": [0, 1, 3], "understand": 0, "let": 0, "u": 0, "consid": 0, "case": 0, "gpio": 0, "ha": 0, "enum": 0, "hal_gpio_out_pin": 0, "output": [0, 2, 4], "member": 0, "shown": [0, 1, 2, 3, 4], "below": [0, 1, 2, 3, 4], "typedef": 0, "index": 0, "0": [0, 1, 3], "hal_gpio_out_pin_0": 0, "hal_gpio_out_pin_1": 0, "hal_gpio_out_pin_2": 0, "hal_gpio_out_pin_3": 0, "hal_gpio_out_pin_4": 0, "hal_gpio_out_pin_5": 0, "6": [0, 5], "hal_gpio_out_pin_6": 0, "7": [0, 5], "hal_gpio_out_pin_7": 0, "8": [0, 5], "hal_gpio_out_pin_8": 0, "9": [0, 4, 5], "hal_gpio_out_pin_9": 0, "total": 0, "hal_gpio_out_pin_max": 0, "To": [0, 1, 3], "map": 0, "real": 0, "we": [0, 1, 3], "structur": 0, "store": 0, "name": 0, "see": [0, 1, 2, 4], "gpioout": 0, "hold": 0, "data": [0, 3, 4], "iomux": 0, "generic_gpio_out_pino_0_iomux": 0, "generic_gpio_out_port": 0, "generic_gpio_out_pino_0_pin": 0, "note": [0, 1, 2, 4, 5], "defin": 0, "ti": 0, "sysconfig": 0, "gener": [0, 1], "file": 0, "line": 0, "dl_gpio_pin_0": 0, "thu": [0, 3], "indirectli": 0, "refer": [0, 1, 2, 3, 4], "advantag": 0, "sinc": [0, 3], "veri": 0, "easi": 0, "chang": [0, 3], "automat": [0, 1, 3], "follow": [0, 1, 3], "without": [0, 3], "code": [0, 2, 4], "As": 0, "seen": 0, "abov": [0, 2, 4], "when": [0, 1, 3], "access": 0, "through": [0, 1, 3], "pass": [0, 3], "drv8889q1": 0, "initi": [0, 3], "its": [0, 1, 3], "assign": 0, "snippet": 0, "dir": [0, 4], "drvoff": [0, 4], "nsleep": [0, 4], "nfault": [0, 2, 4], "hal_gpio_in_pin_0": 0, "vref": [0, 2, 4], "hal_dac_channel_0": 0, "step": [0, 2, 4], "hal_pwm_channel_0": 0, "spi": [0, 5], "hal_spi_channel_0": 0, "spic": 0, "hal_spi_cs_2": 0, "now": 0, "set": [0, 1, 2, 4, 5], "high": 0, "drv8889q1_setnsleep": 0, "void": 0, "drv8889q1_instanc": 0, "handl": [0, 5], "hal_setgpiov": 0, "hal_gpio_value_high": 0, "startup": 0, "delai": 0, "readi": 0, "hal_delaymillisecond": 0, "drv8889q1_spi_ready_delay_m": 0, "interact": 0, "exampl": [0, 1, 3], "where": 0, "same": [0, 2, 4], "concept": 0, "timer": 0, "dac": 0, "free": 0, "expand": 0, "": 0, "featur": 0, "microcontrol": 0, "strongli": 0, "modifi": 0, "exist": 0, "ensur": 0, "migrat": 0, "new": 0, "version": 0, "drive": [0, 1, 3], "task": 0, "simpl": 0, "start": [0, 2, 4, 5], "stop": [0, 1, 3], "speed": [0, 1, 3], "idea": 0, "independ": [0, 3], "befor": 0, "expect": 0, "specifi": 0, "drv": [0, 3], "ain1": [0, 2], "ain2": [0, 2], "bin1": [0, 2], "bin2": [0, 2], "aipropi": [0, 2], "hal_adc_channel_0": 0, "bipropi": [0, 2], "hal_adc_channel_1": 0, "indexertim": 0, "hal_tim_channel_0": 0, "adctriggertim": 0, "hal_tim_channel_1": 0, "along": 0, "also": [0, 3], "logic": [0, 4], "keep": [0, 2, 4], "worri": 0, "intern": 0, "part": 0, "drv8411a_setpwmdr": 0, "h": [0, 1, 2, 3], "bridg": [0, 1, 2, 3], "state": [0, 1, 3], "__static_inlin": 0, "in1": 0, "in2": 0, "stepper_dr": 0, "drv8411a_decai": 0, "decaymod": 0, "hal_gpio_valu": 0, "in1gpiov": 0, "hal_gpio_value_low": 0, "in2gpiov": 0, "switch": 0, "stepper_drive_dis": 0, "break": 0, "stepper_drive_decai": 0, "drv8411a_decay_slow": 0, "els": [0, 1, 3], "could": 0, "each": [0, 1], "correct": 0, "base": [0, 1], "enabl": [0, 1, 3], "multipl": 0, "contain": 0, "algorithm": 0, "help": [0, 3], "reduc": [0, 1], "size": 0, "drv8411a_setstepmod": 0, "drv8411a_inst": 0, "drv8411a_step": 0, "stepmod": 0, "drv8411a_step_full_step": 0, "stepper_setindexerincv": 0, "stepper_step_inc_dec_full_step": 0, "drv8411a_setspe": 0, "setfreq": 0, "drv8411a_step_half_step_nc": 0, "stepper_step_inc_dec_half_step": 0, "stepper_setsteptyp": 0, "stepper_step_type_noncir": 0, "stepper_step_type_cir": 0, "In": [0, 1, 3], "increment": 0, "decrement": 0, "valu": [0, 1, 3], "howev": 0, "more": 0, "doesn": [0, 1, 3], "t": [0, 1, 3], "mode": [0, 4, 5], "them": 0, "unawar": 0, "interfac": [0, 3], "ie": 0, "whether": 0, "pwm": 0, "phase": [0, 2, 4], "turn": [0, 1, 2, 3, 4], "point": 0, "discuss": 0, "wa": 0, "section": 0, "fulli": 0, "function": [0, 3], "manipul": 0, "platform": 0, "pleas": [0, 2, 4], "document": 0, "inform": 0, "offlin": 0, "plot": 0, "sometim": 0, "fail": 0, "program": 0, "load": 0, "current": [0, 2, 4, 5], "cc": 0, "id": 0, "launchpad": [0, 2, 4], "lp": [0, 1, 2, 3, 4], "mspm0l1306": [0, 1, 2, 3, 4], "link": [0, 1, 3], "evm": [0, 1, 2, 3, 4], "found": [1, 3], "here": [1, 3], "allow": [1, 3], "stepper": [1, 2, 3, 4], "motor": [1, 2, 3, 4], "adjust": [1, 3], "direct": [1, 3, 4], "movement": [1, 3], "devic": [1, 3, 5], "well": [1, 3], "statu": [1, 3], "begin": [1, 3], "connect": [1, 2, 3, 4], "necessari": [1, 3], "drv8411aevm": [1, 2], "detail": [1, 3], "plug": [1, 3], "micro": [1, 2, 3, 4], "usb": [1, 2, 3, 4], "cabl": [1, 2, 3, 4], "pc": [1, 2, 3, 4], "power": [1, 2, 3, 4], "suppli": [1, 2, 3, 4], "launch": [1, 3], "try": [1, 3], "establish": [1, 3], "commun": [1, 3], "happen": [1, 3], "disconnect": [1, 3], "click": [1, 3], "icon": [1, 3], "left": [1, 3], "side": [1, 2, 3, 4], "imag": [1, 2, 3], "A": [1, 2, 3, 4], "messag": [1, 3], "show": [1, 3], "target": [1, 3], "displai": [1, 3], "tri": [1, 3], "board": [1, 3, 4], "pane": [1, 3], "locat": [1, 2, 3, 4], "bottom": [1, 3], "success": [1, 3], "appear": [1, 3], "find": [1, 2, 3, 4], "sleep": [1, 3, 4], "wake": [1, 3], "driver": [1, 2, 3, 4], "paramet": [1, 3], "so": [1, 2, 3, 4], "forth": [1, 3], "hover": [1, 3], "over": [1, 3], "widget": [1, 3], "brief": [1, 3], "descript": [1, 3], "desir": [1, 3, 4], "unit": [1, 3], "puls": [1, 3], "per": [1, 3], "second": [1, 3], "know": [1, 3], "rotat": [1, 3], "comput": [1, 3], "rp": [1, 3], "revolut": [1, 3], "200ppr": [1, 3], "200pp": [1, 3], "equat": [1, 3], "1rp": [1, 3], "full": [1, 3], "5rp": [1, 3], "half": [1, 3], "run": [1, 2, 3, 4], "input": [1, 2, 3, 4], "box": [1, 3], "do": [1, 2, 3, 4], "limit": [1, 2, 4, 5], "e": [1, 3], "would": [1, 3], "vibrat": [1, 3], "torqu": [1, 3], "receiv": [1, 3], "move": [1, 3], "enter": [1, 3], "reach": [1, 3], "recircul": [1, 3], "non": 1, "circular": 1, "select": [1, 3, 4], "slow": 1, "fast": 1, "nois": [1, 3], "reduct": 1, "prefer": 1, "top": [1, 3], "right": [1, 2, 3], "screen": [1, 3], "led": [1, 2, 3], "red": [1, 3], "occur": [1, 3], "clear": [1, 3], "button": [1, 3], "send": 1, "factor": 1, "resistor": 1, "proper": 1, "read": [1, 3], "instantan": 1, "flow": 1, "b": [1, 2, 4], "wind": 1, "isena": 1, "isenb": 1, "respect": 1, "trip": 1, "graphic": 1, "view": 1, "captur": 1, "represent": [1, 2, 4], "evalu": [2, 4], "modul": [2, 4], "dual": 2, "regul": 2, "dc": [2, 4], "support": [2, 4, 5], "voltag": [2, 4], "ten": 2, "jumper": [2, 4], "wire": [2, 4], "j9": [2, 4], "j10": [2, 4], "header": [2, 4], "3v": [2, 4], "remov": 2, "signal": [2, 4], "tabl": [2, 4], "schemat": [2, 4], "connector": [2, 4], "pinout": [2, 4], "should": [2, 3], "mspm0l": [2, 4], "pa0": [2, 4], "pa3": [2, 4], "pa1": [2, 4], "pa4": 2, "fault": [2, 4, 5], "pa5": [2, 4], "pa22": [2, 4], "pa27": 2, "pa26": 2, "vdd": 2, "3v3": [2, 4], "vcc": 2, "common": [2, 4], "ground": [2, 4], "gnd": [2, 4], "aout1": [2, 4], "aout2": [2, 4], "bout1": [2, 4], "bout2": [2, 4], "termin": [2, 4], "block": [2, 4], "color": [2, 4], "look": [2, 4], "datasheet": [2, 4], "less": [2, 4], "than": [2, 4], "11v": 2, "off": [2, 4], "posit": [2, 3, 4], "neg": [2, 4], "vm": [2, 4], "pgnd": [2, 4], "yet": [2, 4], "colour": [2, 4], "everi": [2, 4], "verifi": [2, 4], "your": [2, 4], "rate": [2, 4], "higher": [2, 4], "5a": [2, 3, 4], "green": 2, "onc": [2, 4], "complet": [2, 4], "you": [2, 4], "q1evm": [3, 4], "mcu": 3, "default": 3, "written": 3, "temporarili": 3, "activ": 3, "after": 3, "write": 3, "lowest": 3, "level": [3, 4], "except": 3, "smart": 3, "tune": 3, "rippl": 3, "toff": 3, "f": 3, "scale": 3, "further": 3, "fscurrent": 3, "1a": 3, "50": 3, "result": 3, "disabl": [3, 4], "trq": 3, "count": 3, "threshold": 3, "trigger": 3, "option": 3, "rev": 3, "There": 3, "bit": 3, "some": 3, "affect": 3, "address": 3, "bitfield": 3, "bit0": 3, "bit7": 3, "mask": 3, "It": 3, "one": 3, "0x1": 3, "two": 3, "0x11": 3, "0x111": 3, "ctrl3": 3, "automot": 4, "256": 4, "microstep": 4, "stall": [4, 5], "detect": [4, 5], "13": 4, "between": 4, "pin": 4, "11": 4, "j2": 4, "chip": 4, "pa15": 4, "nsc": 4, "sdo": 4, "vsdo": 4, "serial": 4, "pa16": 4, "pa18": 4, "sdi": 4, "clock": 4, "pa6": 4, "sclk": 4, "pa12": 4, "45v": 4, "librari": 5, "overview": 5, "1": 5, "softwar": 5, "2": 5, "api": 5, "3": 5, "known": 5, "issu": 5, "4": 5, "drv8411a": 5, "get": 5, "window": 5, "5": 5, "decai": 5, "monitor": 5, "sens": 5, "setup": 5, "drv8889": 5, "q1": 5, "import": 5, "10": 5, "regist": 5}, "objects": {}, "objtypes": {}, "objnames": {}, "titleterms": {"stepper": [0, 5], "motor": [0, 5], "control": [0, 1, 3, 5], "librari": 0, "overview": 0, "1": [0, 1, 2, 3, 4], "softwar": 0, "applic": 0, "layer": 0, "2": [0, 1, 2, 3, 4], "hal": 0, "modul": 0, "3": [0, 1, 3], "driver": 0, "pin": 0, "associ": 0, "api": 0, "4": [0, 1, 3], "5": [0, 1, 3], "mspm0": 0, "driverlib": 0, "guid": [0, 1, 2, 3, 4, 5], "known": 0, "issu": 0, "support": 0, "devic": 0, "drv8411a": [0, 1, 2], "drv8889": [0, 3, 4], "q1": [0, 3, 4], "gui": [1, 3], "user": [1, 2, 3, 4, 5], "get": [1, 3], "start": [1, 3], "us": [1, 3], "window": [1, 3], "mode": [1, 3], "spin": [1, 3], "step": [1, 3], "decai": [1, 3], "6": [1, 3], "fault": [1, 3], "monitor": [1, 3], "handl": [1, 3], "7": [1, 3], "current": [1, 3], "sens": 1, "ripropi": 1, "aipropi": 1, "isens": 1, "itrip": 1, "phase": 1, "plot": 1, "hardwar": [2, 4], "requir": [2, 4], "setup": [2, 4], "import": 3, "note": 3, "8": 3, "set": 3, "limit": 3, "9": 3, "stall": 3, "detect": 3, "10": 3, "spi": 3, "regist": 3}, "envversion": {"sphinx.domains.c": 2, "sphinx.domains.changeset": 1, "sphinx.domains.citation": 1, "sphinx.domains.cpp": 6, "sphinx.domains.index": 1, "sphinx.domains.javascript": 2, "sphinx.domains.math": 2, "sphinx.domains.python": 3, "sphinx.domains.rst": 2, "sphinx.domains.std": 2, "sphinx": 56}})