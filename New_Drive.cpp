void target(int distance, int radius) {
#if debug == 1
	DEBUG.println("Targeting...");
#endif
	bool ccw = 0;
	int E1Tar;
	int E2Tar;
	if (radius == 0) {
		E1Tar = enc_target(distance);
		E2Tar = E1Tar;
#if debug == 1
		DEBUG.print("Straight Line, length: ");
		DEBUG.println(distance, DEC);
#endif
	}
	else {
		int Do = sweep(distance, abs(radius));
		int Di = sweep(distance, abs(radius), 1);
		int EoTar = enc_target(Do);
		int EiTar = enc_target(Di);
#if debug == 1
		DEBUG.print("Swept Radius, length, radius: ");
		DEBUG.print(distance, DEC);
		DEBUG.print(", ");
		DEBUG.print(radius, DEC);
#endif
		if (radius < 0) {
			E1Tar = EoTar;
			E2Tar = EiTar;
#if debug == 1
			DEBUG.println(" Anti-Clockwise");
#endif
		}
		else {
			E1Tar = EiTar;
			E2Tar = EoTar;
#if debug == 1
			DEBUG.println(" Clockwise");
#endif
		}
	}
	DriveTo(E1Tar, E2Tar);
	return;
}