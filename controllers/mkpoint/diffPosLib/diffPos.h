namespace diffpos{

typedef struct Pos2D Pos2D;

int diff2rad(float* rad, float vr, float vl);

int diff2V(float* vel, float vr, float vl);

void switchOrig(Pos2D* out, Pos2D in, Pos2D newOrg);

void rad2rotCent(Pos2D* cent, Pos2D pos_cur, float radius);

void rotPosByAngle(Pos2D* out, Pos2D pos_cur, float angle);

int transform(Pos2D* out, Pos2D in, float vr, float vl, float time_diff);

int print(Pos2D pos);

}
