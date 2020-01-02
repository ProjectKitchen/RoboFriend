static float motorMapForward[150][3] = {
{1,0.0,0.0},
{2,0.0,0.0},
{3,0.0,0.0},
{4,0.0,0.0},
{5,0.0,0.0},
{6,0.0,26.0},
{7,0.0,26.0},
{8,0.0,34.0},
{9,0.0,38.0},
{10,0.0,51.0},
{11,0.0,51.0},
{12,0.0,54.8},
{13,0.0,58.8},
{14,0.0,64.8},
{15,0.0,60.8},
{16,0.0,69.0},
{17,0.0,64.8},
{18,20.0,68.8},
{19,20.0,70.8},
{20,32.0,74.6},
{21,36.0,70.8},
{22,44.4,76.8},
{23,44.2,72.8},
{24,56.8,74.8},
{25,54.8,78.8},
{26,62.6,80.6},
{27,64.4,80.4},
{28,70.4,84.4},
{29,71.0,83.0},
{30,78.8,90.8},
{31,77.0,89.0},
{32,86.8,88.8},
{33,87.0,91.0},
{34,94.8,90.6},
{35,90.8,90.8},
{36,103.6,90.8},
{37,103.6,93.0},
{38,111.2,97.0},
{39,111.6,93.0},
{40,121.6,97.4},
{41,121.2,99.0},
{42,125.6,101.6},
{43,125.6,97.2},
{44,129.2,103.2},
{45,131.6,103.6},
{46,140.2,103.6},
{47,133.6,105.6},
{48,140.8,108.0},
{49,139.6,107.2},
{50,143.8,107.2},
{51,143.8,109.2},
{52,148.4,113.6},
{53,144.4,111.6},
{54,148.4,111.6},
{55,146.4,113.6},
{56,148.4,115.6},
{57,150.4,115.6},
{58,150.4,115.6},
{59,152.4,121.6},
{60,152.4,121.6},
{61,152.4,119.6},
{62,154.4,123.6},
{63,154.4,123.6},
{64,160.4,125.6},
{65,160.4,125.6},
{66,158.4,125.6},
{67,158.4,125.6},
{68,158.4,131.6},
{69,160.4,131.6},
{70,162.4,131.6},
{71,158.4,131.6},
{72,162.4,133.6},
{73,161.8,135.4},
{74,161.2,134.6},
{75,162.4,136.0},
{76,161.8,139.8},
{77,162.4,140.4},
{78,161.8,141.8},
{79,162.4,142.4},
{80,161.8,141.8},
{81,166.4,146.4},
{82,173.0,147.0},
{83,165.8,147.8},
{84,165.8,147.8},
{85,170.4,148.4},
{86,170.4,148.4},
{87,168.4,152.4},
{88,169.8,151.8},
{89,172.4,156.4},
{90,170.4,156.4},
{91,170.4,152.4},
{92,172.4,156.4},
{93,171.8,155.8},
{94,171.8,159.8},
{95,171.8,161.8},
{96,177.8,161.8},
{97,172.4,162.4},
{98,173.0,165.0},
{99,176.4,164.4},
{100,180.4,168.4},
{101,174.4,166.4},
{102,178.4,170.4},
{103,174.4,172.4},
{104,176.4,174.4},
{105,173.2,171.2},
{106,182.4,172.4},
{107,178.4,174.4},
{108,182.4,176.4},
{109,177.8,177.8},
{110,179.2,179.2},
{111,177.6,179.6},
{112,179.8,177.8},
{113,182.4,178.4},
{114,183.0,181.0},
{115,183.8,181.6},
{116,183.8,181.8},
{117,181.0,181.0},
{118,188.2,186.0},
{119,183.8,181.8},
{120,184.6,186.8},
{121,184.0,184.0},
{122,186.8,184.4},
{123,189.0,184.6},
{124,184.6,182.4},
{125,184.4,182.2},
{126,184.4,182.4},
{127,185.4,183.2},
{128,186.8,186.6},
{129,183.8,181.8},
{130,188.8,184.6},
{131,188.0,186.0},
{132,193.2,186.8},
{133,186.8,184.6},
{134,186.0,184.0},
{135,189.6,185.2},
{136,186.0,184.0},
{137,191.2,182.4},
{138,186.0,181.8},
{139,186.8,182.4},
{140,186.6,182.4},
{141,188.2,183.8},
{142,190.2,181.8},
{143,192.4,181.6},
{144,186.8,184.6},
{145,190.2,181.6},
{146,188.0,184.0},
{147,189.6,187.4},
{148,186.8,184.6},
{149,187.4,183.0},
{150,189.4,181.0},
};

static float motorMapBackward[150][3] = {
{-1,0.0,0.0},
{-2,0.0,0.0},
{-3,0.0,0.0},
{-4,0.0,0.0},
{-5,0.0,0.0},
{-6,0.0,-24.0},
{-7,0.0,-22.0},
{-8,0.0,-34.2},
{-9,0.0,-34.0},
{-10,0.0,-46.6},
{-11,0.0,-44.6},
{-12,0.0,-52.8},
{-13,0.0,-50.8},
{-14,0.0,-60.8},
{-15,0.0,-61.0},
{-16,0.0,-65.0},
{-17,0.0,-61.0},
{-18,-22.0,-64.8},
{-19,-24.0,-67.0},
{-20,-44.6,-67.0},
{-21,-40.0,-71.0},
{-22,-51.0,-71.2},
{-23,-52.8,-70.8},
{-24,-60.8,-74.8},
{-25,-60.8,-70.8},
{-26,-70.8,-78.8},
{-27,-70.8,-74.8},
{-28,-79.0,-79.0},
{-29,-76.8,-78.8},
{-30,-84.8,-80.8},
{-31,-86.6,-82.6},
{-32,-90.8,-82.8},
{-33,-90.8,-80.8},
{-34,-102.0,-87.0},
{-35,-103.6,-86.8},
{-36,-113.6,-86.8},
{-37,-111.6,-90.8},
{-38,-115.6,-90.8},
{-39,-119.6,-90.8},
{-40,-125.6,-93.0},
{-41,-125.2,-97.0},
{-42,-131.6,-99.6},
{-43,-133.8,-101.6},
{-44,-136.4,-93.2},
{-45,-140.8,-102.0},
{-46,-141.8,-99.0},
{-47,-142.4,-103.6},
{-48,-143.2,-104.8},
{-49,-144.4,-103.6},
{-50,-151.0,-108.0},
{-51,-152.4,-103.6},
{-52,-152.4,-107.6},
{-53,-153.0,-106.0},
{-54,-154.4,-109.6},
{-55,-154.4,-111.6},
{-56,-153.8,-111.2},
{-57,-159.8,-111.2},
{-58,-159.8,-115.2},
{-59,-157.8,-117.0},
{-60,-161.2,-118.6},
{-61,-159.2,-116.8},
{-62,-161.8,-119.0},
{-63,-160.6,-116.4},
{-64,-161.8,-121.0},
{-65,-162.4,-123.6},
{-66,-168.4,-125.6},
{-67,-161.8,-125.2},
{-68,-163.2,-124.8},
{-69,-167.0,-124.0},
{-70,-168.4,-125.6},
{-71,-170.4,-129.6},
{-72,-168.4,-129.6},
{-73,-168.4,-131.6},
{-74,-171.0,-132.0},
{-75,-168.4,-131.6},
{-76,-172.4,-133.6},
{-77,-170.4,-140.2},
{-78,-172.4,-136.0},
{-79,-170.6,-136.6},
{-80,-172.4,-140.2},
{-81,-175.0,-140.8},
{-82,-174.4,-142.4},
{-83,-176.4,-144.4},
{-84,-173.8,-145.8},
{-85,-178.4,-144.4},
{-86,-178.4,-146.4},
{-87,-179.0,-147.0},
{-88,-180.4,-148.4},
{-89,-178.4,-148.4},
{-90,-177.8,-151.8},
{-91,-177.8,-151.8},
{-92,-180.4,-152.4},
{-93,-177.8,-151.8},
{-94,-182.4,-156.4},
{-95,-182.4,-156.4},
{-96,-177.8,-157.8},
{-97,-179.8,-155.8},
{-98,-181.0,-161.0},
{-99,-182.4,-158.4},
{-100,-184.6,-162.4},
{-101,-182.4,-160.4},
{-102,-183.8,-167.8},
{-103,-181.8,-163.8},
{-104,-181.8,-163.8},
{-105,-184.6,-162.4},
{-106,-186.0,-169.8},
{-107,-181.8,-165.8},
{-108,-190.2,-171.8},
{-109,-188.0,-171.8},
{-110,-186.6,-178.4},
{-111,-184.6,-176.4},
{-112,-190.2,-179.8},
{-113,-183.8,-173.8},
{-114,-186.0,-177.8},
{-115,-188.8,-176.4},
{-116,-185.8,-181.6},
{-117,-184.0,-173.8},
{-118,-186.0,-179.8},
{-119,-188.2,-181.8},
{-120,-184.6,-182.4},
{-121,-190.2,-181.8},
{-122,-186.0,-179.8},
{-123,-188.8,-182.4},
{-124,-193.2,-182.4},
{-125,-185.2,-179.0},
{-126,-190.2,-181.8},
{-127,-190.2,-179.8},
{-128,-186.6,-182.4},
{-129,-188.8,-182.4},
{-130,-188.2,-181.8},
{-131,-188.0,-181.8},
{-132,-190.2,-181.6},
{-133,-190.4,-181.8},
{-134,-190.4,-181.8},
{-135,-183.8,-181.6},
{-136,-189.6,-183.0},
{-137,-187.4,-181.0},
{-138,-187.4,-181.2},
{-139,-186.8,-182.4},
{-140,-191.0,-182.4},
{-141,-190.2,-181.8},
{-142,-188.8,-180.4},
{-143,-187.4,-183.0},
{-144,-190.4,-181.8},
{-145,-188.8,-180.4},
{-146,-190.2,-181.6},
{-147,-188.0,-181.6},
{-148,-189.6,-179.0},
{-149,-190.2,-181.8},
{-150,-192.4,-181.8},
};

int leftStartDrive=18;
int rightStartDrive=6;
