package com.lushprojects.circuitjs1.client;

import com.google.gwt.i18n.client.NumberFormat;

import com.lushprojects.circuitjs1.client.util.Locale;

// based on https://ctms.engin.umich.edu/CTMS/index.php?example=MotorPosition&section=SystemModeling


class ThreePhaseMotorElm extends CircuitElm {

    double Rs, Rr, Ls, Lr, Lm;
    double b;
    public double angle;
    public double speed;

    double coilCurrent;
    double inertiaCurrent;
    double curcounts[];
    double J;
    Point posts[], leads[];
    
    public ThreePhaseMotorElm(int xx, int yy) { 
	super(xx, yy); 
	Rs = .435;
	Rr = .816;
	Ls = .0294;
	Lr = .0297;
	Lm = .0287;
	J = 1;
	angle = pi/2; speed = filteredSpeed = 0;
	b= 0.05;
        voltSources = new int[2];
        curcounts = new double[3];
        coilCurrents = new double[coilCount];
    }
    public ThreePhaseMotorElm(int xa, int ya, int xb, int yb, int f, StringTokenizer st) {
	super(xa, ya, xb, yb, f);
	angle = pi/2;
	filteredSpeed = speed = 0;
	Rs = new Double(st.nextToken()).doubleValue();
	Rr = new Double(st.nextToken()).doubleValue(); 
	Ls = new Double(st.nextToken()).doubleValue();
	Lr = new Double(st.nextToken()).doubleValue();
	Lm = new Double(st.nextToken()).doubleValue();
	b =  new Double(st.nextToken()).doubleValue();
	try {
	    J = new Double(st.nextToken()).doubleValue();
	} catch (Exception e) { J = 1; }
        voltSources = new int[2];
        curcounts = new double[3];
        coilCurrents = new double[coilCount];
    }
    int getDumpType() { return 427; }
    String dump() {
	// dump: inductance; resistance, K, Kb, J, b, gearRatio, tau
	return super.dump() + " " +  Rs + " " + Rr + " " + Ls + " " +  Lr + " " + Lm + " " + b + " " + J;
    }
    public double getAngle(){ return(angle);}

    Point motorCenter;

    void setPoints() {
	super.setPoints();
	posts = newPointArray(6);
	leads = newPointArray(6);
	int i;
	int q = (Math.abs(dy) > Math.abs(dx)) ? -1 : 1;
	for (i = 0; i != 3; i++) {
	    interpPoint(point1, point2, posts[i*2], 0, -q*32*(i-1));
	    interpPoint(point1, point2, leads[i*2], .45, -q*32*(i-1));
	    interpPoint(point1, point2, posts[i*2+1], 1, q*32*(i-1));
	    interpPoint(point1, point2, leads[i*2+1], .55, q*32*(i-1));
	}
	motorCenter = interpPoint(point1, point2, .5);
	allocNodes();
    }
    int getPostCount() { return 6; }
    Point getPost(int n) { return posts[n]; }
    int getInternalNodeCount() { return 7; }
    int getVoltageSourceCount() { return 2; }
    void reset() {
	super.reset();
	filteredSpeed = speed = 0;
        coilCurSourceValues = new double[coilCount];
        coilCurrents = new double[coilCount];
    }

    final int n001_ind = 6;
    final int n002_ind = 7;
    final int n003_ind = 8;
    final int n004_ind = 9;
    final int n005_ind = 10;
    final int n006_ind = 11;
    final int n007_ind = 12;
    
    final double Zp = 2;
    
    final int coilCount = 5;
    
    // based on https://forum.kicad.info/t/ac-motors-simulation-1-phase-3-phase/14188/3
    
    class ThreePhaseMotorElm extends CircuitElm {
    double Rs, Rr, Ls, Lr, Lm;
    double b;
    public double angle;
    public double speed;

    double coilCurrent;
    double inertiaCurrent;
    double curcounts[];
    double J;
    
    final int coilCount = 6; // 3 del estator + 3 del rotor
    double xformMatrix[][] = new double[coilCount][coilCount];
    double coilInductances[] = new double[coilCount];
    double couplingCoefs[][] = new double[coilCount][coilCount];
    
    public ThreePhaseMotorElm(int xx, int yy) { 
        super(xx, yy); 
        Rs = 0.435;
        Rr = 0.816;
        Ls = 0.0294;
        Lr = 0.0297;
        Lm = 0.0287;
        J = 1;
        angle = Math.PI / 2; 
        speed = 0;
        b = 0.05;
    }

    void setupInductanceMatrix() {
        double Ls_auto = Ls + Lm;
        double Lr_auto = Lr + Lm;
        double Mss = -0.5 * Lm;
        double Mrr = -0.5 * Lm;
        double Msr = Lm;
        
        // Matriz de inductancia L
        for (int i = 0; i < coilCount; i++) {
            for (int j = 0; j < coilCount; j++) {
                xformMatrix[i][j] = 0; // Inicializamos en 0
            }
        }
        
        // Auto-inductancias
        xformMatrix[0][0] = Ls_auto;
        xformMatrix[1][1] = Ls_auto;
        xformMatrix[2][2] = Ls_auto;
        xformMatrix[3][3] = Lr_auto;
        xformMatrix[4][4] = Lr_auto;
        xformMatrix[5][5] = Lr_auto;
        
        // Acoplamientos entre fases del estator
        xformMatrix[0][1] = xformMatrix[1][0] = Mss;
        xformMatrix[0][2] = xformMatrix[2][0] = Mss;
        xformMatrix[1][2] = xformMatrix[2][1] = Mss;
        
        // Acoplamientos entre fases del rotor
        xformMatrix[3][4] = xformMatrix[4][3] = Mrr;
        xformMatrix[3][5] = xformMatrix[5][3] = Mrr;
        xformMatrix[4][5] = xformMatrix[5][4] = Mrr;
        
        // Acoplamientos entre estator y rotor con dependencia angular
        double theta = angle;
        xformMatrix[0][3] = xformMatrix[3][0] = Msr * Math.cos(theta);
        xformMatrix[0][4] = xformMatrix[4][0] = Msr * Math.cos(theta - 2 * Math.PI / 3);
        xformMatrix[0][5] = xformMatrix[5][0] = Msr * Math.cos(theta + 2 * Math.PI / 3);
        xformMatrix[1][3] = xformMatrix[3][1] = Msr * Math.cos(theta + 2 * Math.PI / 3);
        xformMatrix[1][4] = xformMatrix[4][1] = Msr * Math.cos(theta);
        xformMatrix[1][5] = xformMatrix[5][1] = Msr * Math.cos(theta - 2 * Math.PI / 3);
        xformMatrix[2][3] = xformMatrix[3][2] = Msr * Math.cos(theta - 2 * Math.PI / 3);
        xformMatrix[2][4] = xformMatrix[4][2] = Msr * Math.cos(theta + 2 * Math.PI / 3);
        xformMatrix[2][5] = xformMatrix[5][2] = Msr * Math.cos(theta);
    }

    void stamp() {
        setupInductanceMatrix();
        CirSim.invertMatrix(xformMatrix, coilCount);
        
        for (int i = 0; i < coilCount; i++) {
            for (int j = 0; j < coilCount; j++) {
                xformMatrix[i][j] *= sim.timeStep;
                int ni1 = i * 2;
                int nj1 = j * 2;
                int ni2 = i * 2 + 1;
                int nj2 = j * 2 + 1;
                if (i == j) {
                    sim.stampConductance(nodes[ni1], nodes[ni2], xformMatrix[i][i]);
                } else {
                    sim.stampVCCurrentSource(nodes[ni1], nodes[ni2], nodes[nj1], nodes[nj2], xformMatrix[i][j]);
                }
            }
        }
    }
}
    
    int coilNodes[] = { n001_ind, 1, n003_ind, 3, n005_ind, 5, n002_ind, n004_ind, n006_ind, n007_ind };
    double coilCurrents[];
    double coilCurSourceValues[];
    double xformMatrix[][];
    double nodeCurrents[];
    int voltSources[];
    
    void setVoltageSource(int n, int v) { voltSources[n] = v; }
    
    double vs1value, vs2value;

    void startIteration() {
        int i;
        for (i = 0; i != coilCount; i++) {
            double val = coilCurrents[i];
            coilCurSourceValues[i] = val;
        }
        
        double torque = Zp * Math.sqrt(3)/2 * Lm * ((coilCurrents[1]-coilCurrents[2]) * coilCurrents[3] - Math.sqrt(3) * coilCurrents[0] * coilCurrents[4]);
	speed += sim.timeStep * (torque - b * speed)/J;
        angle = angle + speed*sim.timeStep;

        vs1value = -Zp*speed*(Lm*Math.sqrt(3)/2 * (coilCurrents[1]-coilCurrents[2]) + 1.5*Lr*coilCurrents[4]);
        vs2value = Zp*speed*(3/2.*Lm*coilCurrents[0] + 1.5*Lr*coilCurrents[3]);
    }
    
    void doStep() {
        int i;
        for (i = 0; i != coilCount; i++) {
            int n1 = coilNodes[i*2];
            int n2 = coilNodes[i*2+1];
            sim.stampCurrentSource(nodes[n1], nodes[n2], coilCurSourceValues[i]);
        }
	int n002 = nodes[n002_ind];
	int n006 = nodes[n006_ind];
        sim.updateVoltageSource(n002, 0, voltSources[0], -vs1value);
        sim.updateVoltageSource(n006, 0, voltSources[1], -vs2value);
    }
    
    void calculateCurrent() {
        int i;
        int nodeCount = getPostCount() + getInternalNodeCount();
        if (nodeCurrents == null)
            return;
        for (i = 0; i != nodeCount; i++)
            nodeCurrents[i] = 0;
        for (i = 0; i != coilCount; i++) {
            double val = coilCurSourceValues[i];
            if (xformMatrix != null) {
                int j;
                for (j = 0; j != coilCount; j++) {
                    int n1 = coilNodes[j*2];
                    int n2 = coilNodes[j*2+1];
                    double voltdiff = volts[n1]-volts[n2];
                    val += voltdiff*xformMatrix[i][j];
                }
            }
            coilCurrents[i] = val;
            int ni = coilNodes[i];
            nodeCurrents[ni] += val;
            nodeCurrents[ni+1] -= val;
        }
    }

    boolean hasGroundConnection(int n1) {
	return false;
    }
    
    boolean getConnection(int n1, int n2) {
    	return true;
    }
    
    int cr = 37;
    double filteredSpeed;
    
    void draw(Graphics g) {

	int hs = 8;
	setBbox(point1, point2, cr);
	
	int i;
	for (i = 0; i != 6; i++) {
	    setVoltageColor(g, volts[i]);
	    drawThickLine(g, posts[i], leads[i]);
	}
	for (i = 0; i != 3; i++) {
	    curcounts[i] = updateDotCount(coilCurrents[i], curcounts[i]);
	    drawDots(g, posts[i*2]  , leads[i*2],   curcounts[i]);
	    drawDots(g, leads[i*2+1], posts[i*2+1], curcounts[i]);
	}
	
	//getCurrent();
	setPowerColor(g, true);
	Color cc = new Color((int) (165), (int) (165), (int) (165));
	g.setColor(cc);
	g.fillOval(motorCenter.x-(cr), motorCenter.y-(cr), (cr)*2, (cr)*2);
	cc = new Color((int) (10), (int) (10), (int) (10));

	g.setColor(cc);
	double angleAux = Math.round(angle*300.0)/300.0;
	g.fillOval(motorCenter.x-(int)(cr/2.2), motorCenter.y-(int)(cr/2.2), (int)(2*cr/2.2), (int)(2*cr/2.2));

	g.setColor(cc);
	double q = .28*1.7 * 36/dn * 37/27;
	final int gearRatio = 1;
	interpPointFix(point1, point2, ps1, .5 + q*Math.cos(angleAux*gearRatio), q*Math.sin(angleAux*gearRatio));
	interpPointFix(point1, point2, ps2, .5 - q*Math.cos(angleAux*gearRatio), -q*Math.sin(angleAux*gearRatio));

	drawThickerLine(g, ps1, ps2);
	interpPointFix(point1, point2, ps1, .5 + q*Math.cos(angleAux*gearRatio+pi/3), q*Math.sin(angleAux*gearRatio+pi/3));
	interpPointFix(point1, point2, ps2, .5 - q*Math.cos(angleAux*gearRatio+pi/3), -q*Math.sin(angleAux*gearRatio+pi/3));

	drawThickerLine(g, ps1, ps2);

	interpPointFix(point1, point2, ps1, .5 + q*Math.cos(angleAux*gearRatio+2*pi/3), q*Math.sin(angleAux*gearRatio+2*pi/3));
	interpPointFix(point1, point2, ps2, .5 - q*Math.cos(angleAux*gearRatio+2*pi/3), -q*Math.sin(angleAux*gearRatio+2*pi/3));

	drawThickerLine(g, ps1, ps2);

	drawPosts(g);
	
	g.setColor(needsHighlight() ? selectColor : whiteColor);
	g.save();
	if (Math.abs(dy) > Math.abs(dx)) {
	    for (i = 0; i != 3; i++) {
		int d1 = 5;
		g.drawString("UVW".substring(i, i+1) + "1", posts[i*2  ].x+d1, posts[i*2  ].y+8);
		g.drawString("UVW".substring(i, i+1) + "2", posts[i*2+1].x+d1, posts[i*2+1].y-2);		
	    }
	} else {
	    g.context.setTextAlign("center");
	    for (i = 0; i != 3; i++) {
		int d1 = 11;
		int d2 = 7; 
		g.drawString("UVW".substring(i, i+1) + "1", posts[i*2  ].x+d1, posts[i*2  ].y-d2);
		g.drawString("UVW".substring(i, i+1) + "2", posts[i*2+1].x-d1, posts[i*2+1].y-d2);		
	    }
	}
	g.restore();
	filteredSpeed = filteredSpeed*.98 + speed*.02;
    }
    
    static void drawThickerLine(Graphics g, Point pa, Point pb) {
	g.setLineWidth(6.0);
	g.drawLine(pa.x, pa.y, pb.x, pb.y);
	g.setLineWidth(1.0);
    }

    void interpPointFix(Point a, Point b, Point c, double f, double g) {
	int gx = b.y-a.y;
	int gy = a.x-b.x;
	c.x = (int) Math.round(a.x*(1-f)+b.x*f+g*gx);
	c.y = (int) Math.round(a.y*(1-f)+b.y*f+g*gy);
    }


    void getInfo(String arr[]) {
	arr[0] = "3-Phase Motor";
	getBasicInfo(arr);
	arr[3] = Locale.LS("speed") + " = " + getUnitTextRPM(60*Math.abs(filteredSpeed)/(2*Math.PI), Locale.LS("RPM"));
    }
    
    private static String getUnitTextRPM(double v, String u) {
        String sp = " ";
        NumberFormat nf = NumberFormat.getFormat("####.##");
        double va = Math.abs(v);
        if (va < 1e-14)
            // this used to return null, but then wires would display "null" with 0V
            return "0" + sp + u;
        if (va < 1e-9)
            return nf.format(v*1e12) + sp + "p" + u;
        if (va < 1e-6)
            return nf.format(v*1e9) + sp + "n" + u;
        if (va < 1e-3)
            return nf.format(v*1e6) + sp + Locale.muString + u;
        if (va < 1)
            return nf.format(v*1e3) + sp + "m" + u;
        if (va < 1e3)
            return nf.format(v) + sp + u;
        if (va < 1e6)
            return nf.format(v*1e-3) + sp + "k" + u;
        if (va < 1e9)
            return nf.format(v*1e-6) + sp + "M" + u;
        if (va < 1e12)
            return nf.format(v*1e-9) + sp + "G" + u;
        return NumberFormat.getFormat("#.##E000").format(v) + sp + u;
    }

    double getCurrentIntoNode(int n) {
	if (n % 2 == 1)
	    return coilCurrents[n/2];
	return -coilCurrents[n/2];
    }

    public EditInfo getEditInfo(int n) {

	if (n == 0)
	    return new EditInfo("Stator Inductance (H)", Ls, 0, 0);
	if (n == 1)
	    return new EditInfo("Rotor Inductance (H)", Lr, 0, 0);
	if (n == 2)
	    return new EditInfo("Coupling Coefficient", Lm/Math.sqrt(Ls*Lr), 0, 0).setDimensionless();
	if (n == 3)
	    return new EditInfo("Stator Resistance (ohms)", Rs, 0, 0);
	if (n == 4)
	    return new EditInfo("Rotor Resistance (ohms)", Rr, 0, 0);
	if (n == 5)
	    return new EditInfo("Friction coefficient (Nms/rad)", b, 0, 0);
	if (n == 6)
	    return new EditInfo("Moment of inertia (Kg.m^2)", J, 0, 0);
	return null;
    }
    public void setEditValue(int n, EditInfo ei) {

	if (ei.value > 0 && n==0)
	    Ls = ei.value;
	if (ei.value > 0 && n==1)
	    Lr = ei.value;
	if (ei.value > 0 && ei.value < 1 && n==2)
	    Lm = ei.value*Math.sqrt(Ls*Lr);
	if (ei.value > 0 && n==3)
	    Rs = ei.value;
	if (ei.value > 0 && n==4)
	    Rr = ei.value;
	if (n==5)
	    b = ei.value;
	if (ei.value > 0 && n==6)
	    J = ei.value;
    }
    boolean canFlipX() { return false; }
    boolean canFlipY() { return false; }
}
