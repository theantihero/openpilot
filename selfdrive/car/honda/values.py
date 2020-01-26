from cereal import car
from selfdrive.car import dbc_dict

Ecu = car.CarParams.Ecu
VisualAlert = car.CarControl.HUDControl.VisualAlert

# Car button codes
class CruiseButtons:
  RES_ACCEL   = 4
  DECEL_SET   = 3
  CANCEL      = 2
  MAIN        = 1

# See dbc files for info on values"
VISUAL_HUD = {
  VisualAlert.none: 0,
  VisualAlert.fcw: 1,
  VisualAlert.steerRequired: 1,
  VisualAlert.brakePressed: 10,
  VisualAlert.wrongGear: 6,
  VisualAlert.seatbeltUnbuckled: 5,
  VisualAlert.speedTooHigh: 8}

class ECU:
  CAM = 0

class CAR:
  ACCORD = "HONDA ACCORD 2018 SPORT 2T"
  ACCORD_15 = "HONDA ACCORD 2018 LX 1.5T"
  ACCORDH = "HONDA ACCORD 2018 HYBRID TOURING"
  CIVIC = "HONDA CIVIC 2016 TOURING"
  CIVIC_BOSCH = "HONDA CIVIC HATCHBACK 2017 SEDAN/COUPE 2019"
  ACURA_ILX = "ACURA ILX 2016 ACURAWATCH PLUS"
  CLARITY = "HONDA CLARITY 2018 TOURING"
  CRV = "HONDA CR-V 2016 TOURING"
  CRV_5G = "HONDA CR-V 2017 EX"
  CRV_HYBRID = "HONDA CR-V 2019 HYBRID"
  FIT = "HONDA FIT 2018 EX"
  ODYSSEY = "HONDA ODYSSEY 2018 EX-L"
  ODYSSEY_CHN = "HONDA ODYSSEY 2019 EXCLUSIVE CHN"
  ACURA_RDX = "ACURA RDX 2018 ACURAWATCH PLUS"
  PILOT = "HONDA PILOT 2017 TOURING"
  PILOT_2019 = "HONDA PILOT 2019 ELITE"
  RIDGELINE = "HONDA RIDGELINE 2017 BLACK EDITION"

# diag message that in some Nidec cars only appear with 1s freq if VIN query is performed
DIAG_MSGS = {1600: 5, 1601: 8}

FINGERPRINTS = {
  CAR.ACCORD: [{
    148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 399: 7, 419: 8, 420: 8, 427: 3, 432: 7, 441: 5, 446: 3, 450: 8, 464: 8, 477: 8, 479: 8, 495: 8, 545: 6, 662: 4, 773: 7, 777: 8, 780: 8, 804: 8, 806: 8, 808: 8, 829: 5, 862: 8, 884: 8, 891: 8, 927: 8, 929: 8, 1302: 8, 1600: 5, 1601: 8, 1652: 8
  }],
  CAR.ACCORD_15: [{
    148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 427: 3, 432: 7, 441: 5, 446: 3, 450: 8, 464: 8, 477: 8, 479: 8, 495: 8, 545: 6, 662: 4, 773: 7, 777: 8, 780: 8, 804: 8, 806: 8, 808: 8, 829: 5, 862: 8, 884: 8, 891: 8, 927: 8, 929: 8, 1302: 8, 1600: 5, 1601: 8, 1652: 8
  }],
  CAR.ACCORDH: [{
    148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 387: 8, 388: 8, 399: 7, 419: 8, 420: 8, 427: 3, 432: 7, 441: 5, 450: 8, 464: 8, 477: 8, 479: 8, 495: 8, 525: 8, 545: 6, 662: 4, 773: 7, 777: 8, 780: 8, 804: 8, 806: 8, 808: 8, 829: 5, 862: 8, 884: 8, 891: 8, 927: 8, 929: 8, 1302: 8, 1600: 5, 1601: 8, 1652: 8
  }],
  CAR.ACURA_ILX: [{
    57: 3, 145: 8, 228: 5, 304: 8, 316: 8, 342: 6, 344: 8, 380: 8, 398: 3, 399: 7, 419: 8, 420: 8, 422: 8, 428: 8, 432: 7, 464: 8, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 542: 7, 545: 4, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 829: 5, 882: 2, 884: 7, 887: 8, 888: 8, 892: 8, 923: 2, 929: 4, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1030: 5, 1034: 5, 1036: 8, 1039: 8, 1057: 5, 1064: 7, 1108: 8, 1365: 5,
  }],
  # Acura RDX w/ Added Comma Pedal Support (512L & 513L)
  CAR.ACURA_RDX: [{
    57: 3, 145: 8, 229: 4, 308: 5, 316: 8, 342: 6, 344: 8, 380: 8, 392: 6, 398: 3, 399: 6, 404: 4, 420: 8, 422: 8, 426: 8, 432: 7, 464: 8, 474: 5, 476: 4, 487: 4, 490: 8, 506: 8, 512: 6, 513: 6, 542: 7, 545: 4, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 829: 5, 882: 2, 884: 7, 887: 8, 888: 8, 892: 8, 923: 2, 929: 4, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1033: 5, 1034: 5, 1036: 8, 1039: 8, 1057: 5, 1064: 7, 1108: 8, 1365: 5, 1424: 5, 1729: 1
  }],
  CAR.CIVIC: [{
    57: 3, 148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 427: 3, 428: 8, 432: 7, 450: 8, 464: 8, 470: 2, 476: 7, 487: 4, 490: 8, 493: 5, 506: 8, 512: 6, 513: 6, 545: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 829: 5, 862: 8, 884: 8, 891: 8, 892: 8, 927: 8, 929: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1108: 8, 1302: 8, 1322: 5, 1361: 5, 1365: 5, 1424: 5, 1633: 8,
  }],
  CAR.CIVIC_BOSCH: [{
  # 2017 Civic Hatchback EX, 2019 Civic Sedan Touring Canadian, and 2018 Civic Hatchback Executive Premium 1.0L CVT European
    57: 3, 148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 427: 3, 428: 8, 432: 7, 441: 5, 450: 8, 460: 3, 464: 8, 470: 2, 476: 7, 477: 8, 479: 8, 490: 8, 493: 5, 495: 8, 506: 8, 545: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 829: 5, 862: 8, 884: 8, 891: 8, 892: 8, 927: 8, 929: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1108: 8, 1302: 8, 1322: 5, 1361: 5, 1365: 5, 1424: 5, 1600: 5, 1601: 8, 1625: 5, 1629: 5, 1633: 8,
  }],
  CAR.CLARITY: [{
    57: 3, 304: 8, 312: 8, 315: 7, 330: 8, 344: 8, 380: 8, 387: 8, 388: 8, 409: 8, 419: 8, 420: 8, 427: 3, 428: 8, 432: 7, 441: 5, 450: 8, 464: 8, 476: 8, 478: 3, 506: 8, 538: 5, 545: 5, 547: 6, 559: 3, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 815: 8, 829: 5, 831: 5, 832: 3, 833: 8, 856: 7, 862: 8, 884: 8, 891: 8, 900: 8, 904: 8, 905: 8, 906: 4, 923: 2, 927: 8, 929: 8, 976: 8, 983: 8, 1029: 8, 1036: 8, 1070: 8, 1072: 4, 1092: 1, 1108: 8, 1113: 8, 1114: 2, 1125: 8, 1128: 8, 1129: 8, 1302: 8, 1331: 8, 1332: 5, 1341: 5
  }],
  CAR.CRV: [{
    57: 3, 145: 8, 316: 8, 340: 8, 342: 6, 344: 8, 380: 8, 398: 3, 399: 6, 401: 8, 404: 4, 420: 8, 422: 8, 426: 8, 432: 7, 464: 8, 474: 5, 476: 4, 487: 4, 490: 8, 493: 3, 506: 8, 507: 1, 512: 6, 513: 6, 542: 7, 545: 4, 597: 8, 660: 8, 661: 4, 773: 7, 777: 8, 780: 8, 800: 8, 804: 8, 808: 8, 829: 5, 882: 2, 884: 7, 888: 8, 891: 8, 892: 8, 923: 2, 929: 8, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1033: 5, 1036: 8, 1039: 8, 1057: 5, 1064: 7, 1108: 8, 1125: 8, 1296: 8, 1365: 5, 1424: 5, 1600: 5, 1601: 8,
  }],
  CAR.CRV_5G: [{
    57: 3, 148: 8, 199: 4, 228: 5, 231: 5, 232: 7, 304: 8, 330: 8, 340: 8, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 423: 2, 427: 3, 428: 8, 432: 7, 441: 5, 446: 3, 450: 8, 464: 8, 467: 2, 469: 3, 470: 2, 474: 8, 476: 7, 477: 8, 479: 8, 490: 8, 493: 5, 495: 8, 507: 1, 545: 6, 597: 8, 661: 4, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 814: 4, 815: 8, 817: 4, 825: 4, 829: 5, 862: 8, 881: 8, 882: 4, 884: 8, 888: 8, 891: 8, 927: 8, 918: 7, 929: 8, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1108: 8, 1092: 1, 1115: 2, 1125: 8, 1127: 2, 1296: 8, 1302: 8, 1322: 5, 1361: 5, 1365: 5, 1424: 5, 1600: 5, 1601: 8, 1618: 5, 1633: 8, 1670: 5
  }],
  CAR.CRV_HYBRID: [{
    57: 3, 148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 387: 8, 388: 8, 399: 7, 408: 6, 415: 6, 419: 8, 420: 8, 427: 3, 428: 8, 432: 7, 441: 5, 450: 8, 464: 8, 477: 8, 479: 8, 490: 8, 495: 8, 525: 8, 531: 8, 545: 6, 662: 4, 773: 7, 777: 8, 780: 8, 804: 8, 806: 8, 808: 8, 814: 4, 829: 5, 833: 6, 862: 8, 884: 8, 891: 8, 927: 8, 929: 8, 930: 8, 931: 8, 1302: 8, 1361: 5, 1365: 5, 1600: 5, 1601: 8, 1626: 5, 1627: 5
  }],
  CAR.FIT: [{
    57: 3, 145: 8, 228: 5, 304: 8, 342: 6, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 422: 8, 427: 3, 428: 8, 432: 7, 464: 8, 487: 4, 490: 8, 506: 8, 597: 8, 660: 8, 661: 4, 773: 7, 777: 8, 780: 8, 800: 8, 804: 8, 808: 8, 829: 5, 862: 8, 884: 7, 892: 8, 929: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1108: 8, 1322: 5, 1361: 5, 1365: 5, 1424: 5, 1600: 5, 1601: 8
  }],
  # 2018 Odyssey w/ Added Comma Pedal Support (512L & 513L)
  CAR.ODYSSEY: [{
    57: 3, 148: 8, 228: 5, 229: 4, 316: 8, 342: 6, 344: 8, 380: 8, 399: 7, 411: 5, 419: 8, 420: 8, 427: 3, 432: 7, 450: 8, 463: 8, 464: 8, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 542: 7, 545: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 817: 4, 819: 7, 821: 5, 825: 4, 829: 5, 837: 5, 856: 7, 862: 8, 871: 8, 881: 8, 882: 4, 884: 8, 891: 8, 892: 8, 905: 8, 923: 2, 927: 8, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1029: 8, 1036: 8, 1052: 8, 1064: 7, 1088: 8, 1089: 8, 1092: 1, 1108: 8, 1110: 8, 1125: 8, 1296: 8, 1302: 8, 1600: 5, 1601: 8, 1612: 5, 1613: 5, 1614: 5, 1615: 8, 1616: 5, 1619: 5, 1623: 5, 1668: 5
  },
  # 2018 Odyssey Elite w/ Added Comma Pedal Support (512L & 513L)
  {
    57: 3, 148: 8, 228: 5, 229: 4, 304: 8, 342: 6, 344: 8, 380: 8, 399: 7, 411: 5, 419: 8, 420: 8, 427: 3, 432: 7, 440: 8, 450: 8, 463: 8, 464: 8, 476: 4, 490: 8, 506: 8, 507: 1, 542: 7, 545: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 817: 4, 819: 7, 821: 5, 825: 4, 829: 5, 837: 5, 856: 7, 862: 8, 871: 8, 881: 8, 882: 4, 884: 8, 891: 8, 892: 8, 905: 8, 923: 2, 927: 8, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1029: 8, 1036: 8, 1052: 8, 1064: 7, 1088: 8, 1089: 8, 1092: 1, 1108: 8, 1110: 8, 1125: 8, 1296: 8, 1302: 8, 1600: 5, 1601: 8, 1612: 5, 1613: 5, 1614: 5, 1616: 5, 1619: 5, 1623: 5, 1668: 5
  }],
  CAR.ODYSSEY_CHN: [{
    57: 3, 145: 8, 316: 8, 342: 6, 344: 8, 380: 8, 398: 3, 399: 7, 401: 8, 404: 4, 411: 5, 420: 8, 422: 8, 423: 2, 426: 8, 432: 7, 450: 8, 464: 8, 490: 8, 506: 8, 507: 1, 512: 6, 513: 6, 597: 8, 610: 8, 611: 8, 612: 8, 617: 8, 660: 8, 661: 4, 773: 7, 780: 8, 804: 8, 808: 8, 829: 5, 862: 8, 884: 7, 892: 8, 923: 2, 929: 8, 1030: 5, 1137: 8, 1302: 8, 1348: 5, 1361: 5, 1365: 5, 1600: 5, 1601: 8, 1639: 8
  }],
  # 2017 Pilot Touring AND 2016 Pilot EX-L w/ Added Comma Pedal Support (512L & 513L)
  CAR.PILOT: [{
    57: 3, 145: 8, 228: 5, 229: 4, 308: 5, 316: 8, 334: 8, 339: 7, 342: 6, 344: 8, 379: 8, 380: 8, 392: 6, 399: 7, 419: 8, 420: 8, 422: 8, 425: 8, 426: 8, 427: 3, 432: 7, 463: 8, 464: 8, 476: 4, 490: 8, 506: 8, 507: 1, 512: 6, 513: 6, 538: 3, 542: 7, 545: 5, 546: 3, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 829: 5, 837: 5, 856: 7, 871: 8, 882: 2, 884: 7, 891: 8, 892: 8, 923: 2, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1088: 8, 1089: 8, 1108: 8, 1125: 8, 1296: 8, 1424: 5, 1600: 5, 1601: 8, 1612: 5, 1613: 5, 1616: 5, 1618: 5, 1668: 5
  }],
  # this fingerprint also includes the Passport 2019
  CAR.PILOT_2019: [{
    57: 3, 145: 8, 228: 5, 308: 5, 316: 8, 334: 8, 342: 6, 344: 8, 379: 8, 380: 8, 399: 7, 411: 5, 419: 8, 420: 8, 422: 8, 425: 8, 426: 8, 427: 3, 432: 7, 463: 8, 464: 8, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 538: 3, 542: 7, 545: 5, 546: 3, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 808: 8, 817: 4, 819: 7, 821: 5, 825: 4, 829: 5, 837: 5, 856: 7, 871: 8, 881: 8, 882: 2, 884: 7, 891: 8, 892: 8, 923: 2, 927: 8, 929: 8, 983: 8, 985: 3, 1029: 8, 1052: 8, 1064: 7, 1088: 8, 1089: 8, 1092: 1, 1108: 8, 1110: 8, 1125: 8, 1296: 8, 1424: 5, 1445: 8, 1600: 5, 1601: 8, 1612: 5, 1613: 5, 1614: 5, 1615: 8, 1616: 5, 1617: 8, 1618: 5, 1623: 5, 1668: 5
  },
  # 2019 Pilot EX-L
  {
    57: 3, 145: 8, 228: 5, 229: 4, 308: 5, 316: 8, 339: 7, 342: 6, 344: 8, 380: 8, 392: 6, 399: 7, 411: 5, 419: 8, 420: 8, 422: 8, 425: 8, 426: 8, 427: 3, 432: 7, 464: 8, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 542: 7, 545: 5, 546: 3, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 808: 8, 817: 4, 819: 7, 821: 5, 829: 5, 871: 8, 881: 8, 882: 2, 884: 7, 891: 8, 892: 8, 923: 2, 927: 8, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1027: 5, 1029: 8, 1039: 8, 1064: 7, 1088: 8, 1089: 8, 1092: 1, 1108: 8, 1125: 8, 1296: 8, 1424: 5, 1445: 8, 1600: 5, 1601: 8, 1612: 5, 1613: 5, 1616: 5, 1617: 8, 1618: 5, 1623: 5, 1668: 5
  }],
  # Ridgeline w/ Added Comma Pedal Support (512L & 513L)
  CAR.RIDGELINE: [{
    57: 3, 145: 8, 228: 5, 229: 4, 308: 5, 316: 8, 339: 7, 342: 6, 344: 8, 380: 8, 392: 6, 399: 7, 419: 8, 420: 8, 422: 8, 425: 8, 426: 8, 427: 3, 432: 7, 464: 8, 471: 3, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 545: 5, 546: 3, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 829: 5, 871: 8, 882: 2, 884: 7, 892: 8, 923: 2, 927: 8, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1088: 8, 1089: 8, 1108: 8, 1125: 8, 1296: 8, 1365: 5, 1424: 5, 1600: 5, 1601: 8, 1613: 5, 1616: 5, 1618: 5, 1668: 5, 2015: 3
  },
  # 2019 Ridgeline
  {
    57: 3, 145: 8, 228: 5, 229: 4, 308: 5, 316: 8, 339: 7, 342: 6, 344: 8, 380: 8, 392: 6, 399: 7, 419: 8, 420: 8, 422:8, 425: 8, 426: 8, 427: 3, 432: 7, 464: 8, 476: 4, 490: 8, 545: 5, 546: 3, 597: 8, 660: 8, 773: 7, 777: 8, 795: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 871: 8, 882: 2, 884: 7, 892: 8, 923: 2, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1088: 8, 1089: 8, 1092: 1, 1108: 8, 1125: 8, 1296: 8, 1365: 5, 424: 5, 1613: 5, 1616: 5, 1618: 5, 1623: 5, 1668: 5
  }]
}

# add DIAG_MSGS to fingerprints
for c in FINGERPRINTS:
  for f, _ in enumerate(FINGERPRINTS[c]):
    for d in DIAG_MSGS:
      FINGERPRINTS[c][f][d] = DIAG_MSGS[d]

# TODO: Figure out what is relevant
FW_VERSIONS = {
  CAR.CIVIC: {
    (Ecu.unknown, 0x18da10f1, None): [b'37805-5AA-L660\x00\x00'],
    (Ecu.unknown, 0x18da1ef1, None): [b'28101-5CG-A050\x00\x00'],
    (Ecu.unknown, 0x18da28f1, None): [b'57114-TBA-A550\x00\x00'],
    (Ecu.eps, 0x18da30f1, None): [b'39990-TBA-A030\x00\x00', b'39990-TBA,A030\x00\x00'],
    (Ecu.unknown, 0x18da53f1, None): [b'77959-TBA-A030\x00\x00'],
    (Ecu.unknown, 0x18da60f1, None): [b'78109-TBC-A310\x00\x00'],
    (Ecu.unknown, 0x18dab0f1, None): [b'36161-TBC-A030\x00\x00'],
    (Ecu.unknown, 0x18daeff1, None): [b'38897-TBA-A020\x00\x00'],

  },
  CAR.ACCORD: {
    (Ecu.unknown, 0x18da10f1, None): [b'37805-6B2-A650\x00\x00'],
    (Ecu.unknown, 0x18da0bf1, None): [b'54008-TVC-A910\x00\x00'],
    (Ecu.unknown, 0x18da1ef1, None): [b'28102-6B8-A560\x00\x00'],
    (Ecu.unknown, 0x18da2bf1, None): [b'46114-TVA-A060\x00\x00'],
    (Ecu.unknown, 0x18da28f1, None): [b'57114-TVA-C050\x00\x00'],
    (Ecu.eps, 0x18da30f1, None): [b'39990-TVA-A150\x00\x00'],
    (Ecu.unknown, 0x18da3af1, None): [b'39390-TVA-A020\x00\x00'],
    (Ecu.unknown, 0x18da53f1, None): [b'77959-TVA-A460\x00\x00'],
    (Ecu.unknown, 0x18da60f1, None): [b'78109-TVC-A210\x00\x00'],
    (Ecu.unknown, 0x18da61f1, None): [b'78209-TVA-A010\x00\x00'],
    (Ecu.unknown, 0x18dab0f1, None): [b'36802-TVA-A160\x00\x00'],
    (Ecu.unknown, 0x18dab5f1, None): [b'36161-TVA-A060\x00\x00'],
    (Ecu.unknown, 0x18daeff1, None): [b'38897-TVA-A010\x00\x00'],
  }
}

DBC = {
  CAR.ACCORD: dbc_dict('honda_accord_s2t_2018_can_generated', None),
  CAR.ACCORD_15: dbc_dict('honda_accord_lx15t_2018_can_generated', None),
  CAR.ACCORDH: dbc_dict('honda_accord_s2t_2018_can_generated', None),
  CAR.ACURA_ILX: dbc_dict('acura_ilx_2016_can_generated', 'acura_ilx_2016_nidec'),
  CAR.ACURA_RDX: dbc_dict('acura_rdx_2018_can_generated', 'acura_ilx_2016_nidec'),
  CAR.CIVIC: dbc_dict('honda_civic_touring_2016_can_generated', 'acura_ilx_2016_nidec'),
  CAR.CIVIC_BOSCH: dbc_dict('honda_civic_hatchback_ex_2017_can_generated', None),
  CAR.CLARITY: dbc_dict('honda_clarity_hybrid_2018_can_generated', 'acura_ilx_2016_nidec'),
  CAR.CRV: dbc_dict('honda_crv_touring_2016_can_generated', 'acura_ilx_2016_nidec'),
  CAR.CRV_5G: dbc_dict('honda_crv_ex_2017_can_generated', None),
  CAR.CRV_HYBRID: dbc_dict('honda_crv_hybrid_2019_can_generated', None),
  CAR.FIT: dbc_dict('honda_fit_ex_2018_can_generated', 'acura_ilx_2016_nidec'),
  CAR.ODYSSEY: dbc_dict('honda_odyssey_exl_2018_generated', 'acura_ilx_2016_nidec'),
  CAR.ODYSSEY_CHN: dbc_dict('honda_odyssey_extreme_edition_2018_china_can_generated', 'acura_ilx_2016_nidec'),
  CAR.PILOT: dbc_dict('honda_pilot_touring_2017_can_generated', 'acura_ilx_2016_nidec'),
  CAR.PILOT_2019: dbc_dict('honda_pilot_touring_2017_can_generated', 'acura_ilx_2016_nidec'),
  CAR.RIDGELINE: dbc_dict('honda_ridgeline_black_edition_2017_can_generated', 'acura_ilx_2016_nidec'),
}

STEER_THRESHOLD = {
  CAR.ACCORD: 1200,
  CAR.ACCORD_15: 1200,
  CAR.ACCORDH: 1200,
  CAR.ACURA_ILX: 1200,
  CAR.ACURA_RDX: 400,
  CAR.CIVIC: 1200,
  CAR.CIVIC_BOSCH: 1200,
  CAR.CLARITY: 1200,
  CAR.CRV: 1200,
  CAR.CRV_5G: 1200,
  CAR.CRV_HYBRID: 1200,
  CAR.FIT: 1200,
  CAR.ODYSSEY: 1200,
  CAR.ODYSSEY_CHN: 1200,
  CAR.PILOT: 1200,
  CAR.PILOT_2019: 1200,
  CAR.RIDGELINE: 1200,
}

SPEED_FACTOR = {
  CAR.ACCORD: 1.,
  CAR.ACCORD_15: 1.,
  CAR.ACCORDH: 1.,
  CAR.ACURA_ILX: 1.,
  CAR.ACURA_RDX: 1.,
  CAR.CIVIC: 1.,
  CAR.CIVIC_BOSCH: 1.,
  CAR.CLARITY: 1.,
  CAR.CRV: 1.025,
  CAR.CRV_5G: 1.025,
  CAR.CRV_HYBRID: 1.025,
  CAR.FIT: 1.,
  CAR.ODYSSEY: 1.,
  CAR.ODYSSEY_CHN: 1.,
  CAR.PILOT: 1.,
  CAR.PILOT_2019: 1.,
  CAR.RIDGELINE: 1.,
}

# This message sends car info to the radar that is specific to the model. You
# can determine this message by monitoring the OEM system.
# NOT SURE IF WE NEED THESE
VEHICLE_STATE_MSG = {
  CAR.ACURA_ILX: b"\x0f\x18\x51\x02\x5a\x00\x00",
  CAR.ACURA_RDX: b"\x0f\x57\x4f\x02\x5a\x00\x00",
  CAR.CIVIC: b"\x02\x38\x44\x32\x4f\x00\x00",
  CAR.CLARITY: b"\x00\x00\x5d\x02\x5f\x00\x00",
  CAR.CRV: b"\x00\x00\x50\x02\x51\x00\x00",
  CAR.ODYSSEY: b"\x00\x00\x56\x02\x55\x00\x00",
  CAR.PILOT: b"\x00\x00\x56\x02\x58\x00\x00",
  CAR.PILOT_2019: b"\x00\x00\x58\x02\x5c\x00\x00",
  CAR.RIDGELINE: b"\x00\x00\x56\x02\x57\x00\x00",
}

# msgs sent for steering controller by camera module on can 0.
# those messages are mutually exclusive on CRV and non-CRV cars
ECU_FINGERPRINT = {
  ECU.CAM: [0xE4, 0x194],   # steer torque cmd
}

HONDA_BOSCH = [CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH, CAR.CIVIC_BOSCH, CAR.CRV_5G, CAR.CRV_HYBRID]
