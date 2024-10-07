
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <math.h>
#include <algorithm>
#include <cstdlib>
#include <map>
using namespace std;

double lo[3];
double hi[3];
double ds;
double padding;

struct kk
{
    long long idx;
    long long new_idx;
}tmp[40000000];

double xx[8000000], yy[8000000], zz[8000000];

long long tet_idx[10000000][4];

long long getHash(long long a, long long b, long long c)
{
    return a * 16000000ll + b * 4000ll + c;
}

bool cmp(kk a, kk b)
{
    return a.idx < b.idx;
}
map<long long, long long> mapKey, mapKeyInverse;

void pretreat()
{
    freopen("armadillo.obj", "r", stdin);
    freopen("arm.obj", "w", stdout);
    char a[20];
    double xx, yy, zz;
    while (scanf("%s", a) != EOF)
    {
        if (a[0] == 'v')
        {
            scanf("%lf%lf%lf", &xx, &yy, &zz);
            double xx_old = xx;
            double yy_old = yy;
            double zz_old = zz;
            xx = 1.0666666666666666666 * xx_old - 0.26666666666666666665 * yy_old - 0.11764705882352941176 * zz_old;
            yy = -0.26666666666666666666 * xx_old + 1.0666666666666666666 * yy_old - 0.11764705882352941176 * zz_old;
            zz = 0.58823529411764705882 * zz_old;
            printf("v   %.10lf   %.10lf   %.10lf\n", xx, yy, zz);
        }
        else
        {
            int x, y, z;
            scanf("%d%d%d", &x, &y, &z);
            printf("f   %d   %d   %d\n", x, y, z);
        }
    }
}
int main() {

    if (false)//(true)
    {
        pretreat();
        return 0;
    }
    cin >> ds;

    float xds, yds, zds;
    cin >> xds >> yds >> zds;
    padding = ds * 0.1f;

    char s[20];
    double x, y, z;
    int idx = 0;
    freopen("armadillo24K.obj", "r", stdin);
    //freopen("test.txt", "r", stdin);
    while (scanf("%s%lf%lf%lf", s, &x, &y, &z) != EOF)
    {
        // cout << s << endl;
        if (idx == 0)
        {
            lo[0] = x;
            lo[1] = y;
            lo[2] = z;
        }
        else
        {
            lo[0] = min(x, lo[0]);
            lo[1] = min(y, lo[1]);
            lo[2] = min(z, lo[2]);
        }
        xx[idx] = x; yy[idx] = y; zz[idx] = z;
        idx++;
    }
    printf("%d  %.5lf %.5lf %.5lf\n", idx, lo[0], lo[1], lo[2]);

    for (int i = 0; i < idx; i++)
    {
        long long x_int = (xx[i] + padding - lo[0]) / xds;
        long long y_int = (yy[i] + padding - lo[1]) / yds;
        long long z_int = (zz[i] + padding - lo[2]) / zds;

        // cout << x_int << ' ' << y_int << ' ' << z_int << endl;

        if(true)//((z_int) % 2 == 1)
        { 
            tet_idx[i * 6][0] = getHash(x_int, y_int, z_int);
            tet_idx[i * 6][1] = getHash(x_int + 1, y_int, z_int);
            tet_idx[i * 6][2] = getHash(x_int, y_int + 1, z_int);
            tet_idx[i * 6][3] = getHash(x_int, y_int, z_int + 1);

            tet_idx[i * 6 + 1][0] = getHash(x_int + 1, y_int + 1, z_int + 1);
            tet_idx[i * 6 + 1][1] = getHash(x_int + 1, y_int + 1, z_int);
            tet_idx[i * 6 + 1][2] = getHash(x_int, y_int + 1, z_int + 1);
            tet_idx[i * 6 + 1][3] = getHash(x_int + 1, y_int, z_int + 1);

            tet_idx[i * 6 + 2][0] = getHash(x_int, y_int + 1, z_int + 1);
            tet_idx[i * 6 + 2][1] = getHash(x_int, y_int + 1, z_int);
            tet_idx[i * 6 + 2][2] = getHash(x_int + 1, y_int, z_int);
            tet_idx[i * 6 + 2][3] = getHash(x_int, y_int, z_int + 1);

            tet_idx[i * 6 + 3][0] = getHash(x_int + 1, y_int + 1, z_int);
            tet_idx[i * 6 + 3][1] = getHash(x_int, y_int + 1, z_int);
            tet_idx[i * 6 + 3][2] = getHash(x_int + 1, y_int, z_int);
            tet_idx[i * 6 + 3][3] = getHash(x_int, y_int + 1, z_int + 1);

            tet_idx[i * 6 + 4][0] = getHash(x_int + 1, y_int, z_int + 1);
            tet_idx[i * 6 + 4][1] = getHash(x_int, y_int + 1, z_int + 1);
            tet_idx[i * 6 + 4][2] = getHash(x_int, y_int, z_int + 1);
            tet_idx[i * 6 + 4][3] = getHash(x_int + 1, y_int, z_int);

            tet_idx[i * 6 + 5][0] = getHash(x_int + 1, y_int, z_int + 1);
            tet_idx[i * 6 + 5][1] = getHash(x_int, y_int + 1, z_int + 1);
            tet_idx[i * 6 + 5][2] = getHash(x_int + 1, y_int + 1, z_int);
            tet_idx[i * 6 + 5][3] = getHash(x_int + 1, y_int, z_int);
        }
    }
    for (int i = 0; i < idx * 6; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            tmp[i * 4 + j].idx = tet_idx[i][j];
            tmp[i * 4 + j].new_idx = 0;
        }
    }
    sort(tmp, tmp + idx * 6 * 4, cmp);
    int idx_max = 0;
    for (int i = 1; i < idx * 6 * 4; i++)
    {
        //cout << tmp[i].idx << ' ' << tmp[i].new_idx << endl;

        if (tmp[i].idx == tmp[i - 1].idx)
            tmp[i].new_idx = tmp[i - 1].new_idx;
        else
        {
            tmp[i].new_idx = tmp[i - 1].new_idx + 1;
            mapKey[tmp[i - 1].idx] = tmp[i - 1].new_idx;
            mapKeyInverse[tmp[i - 1].new_idx] = tmp[i - 1].idx;
            idx_max++;
        }
        if (i == idx * 24 - 1)
        {
            mapKey[tmp[i].idx] = tmp[i].new_idx;
            mapKeyInverse[tmp[i].new_idx] = tmp[i].idx;
        }

    }
    cout << idx_max << endl;
    freopen("output_vertex.node", "w", stdout);
    cout << idx_max + 1 << " 3 0 0" << endl;
    for (int j = 0; j <= idx_max; j++)
    {
        long long inverseKey = mapKeyInverse[j];
        double xx_f = inverseKey / 16000000;
        double yy_f = (inverseKey - (inverseKey / 16000000 * 16000000)) / 4000;
        double zz_f = inverseKey % 4000;

        long long yy_ff = zz_f;
        double offset = (yy_ff % 2 == 1) ? 0 : (ds / 2.0f);
       /* prlong longf("%d %.10lf %.10lf %.10lf\n",
            j,
            xx_f * ds - 0.5f * ds + lo[0] + offset,
            yy_f * ds - 0.5f * ds + lo[1],
            zz_f * sqrt(3) / 2.0f * ds - 0.5f * ds + lo[2]);*/

        printf("%d %.10lf %.10lf %.10lf\n",
            j,
            xx_f* ds + lo[0] + zz_f * ds * 0.25f + yy_f * ds * 0.25f,
            yy_f* ds - 0.5f * ds + lo[1] + zz_f * ds * 0.25f + xx_f * ds * 0.25f,
            zz_f* sqrt(3) / 2.0f * ds - 0.5f * ds + lo[2]);
    }

    freopen("output_vertex.ele", "w", stdout);
    cout << idx * 6 << " 4 0" << endl;
    for (long long i = 0; i < idx * 6; i++)
    {
        printf("%lld %lld %lld %lld %lld\n",
            i,
            mapKey[tet_idx[i][0]],
            mapKey[tet_idx[i][1]],
            mapKey[tet_idx[i][2]],
            mapKey[tet_idx[i][3]]
        );
    }
    return 0;
}