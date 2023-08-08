#include "model_wrapper.h"
#include "localize_fkd.h"
using namespace std;

int main() {

    localize_fkd::Buffer be(20);

    for (int i = 0; i < 27; i++) {
        std::cout << i << std::flush;
        be.add({i, i});
    }
    std::cout << "finished adding to bif" << std::endl;

    for (int i = 0; i < 20; i++) {
        std::cout << be.b[i][0] << " ";
    }std::cout  << std::endl;

    float f[20];
    be.last_n(f, 20);

    for (int i = 0; i < 20; i++) {
        std::cout << f[i] << " " << std::flush;
    }std::cout << std::endl;

    model::ModelWrapper model("/home/arnav/landmark/src/localize_fkd/arnav_model_trace.pt");
    float in[90] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.7880923748016357,0.7880923748016357,0.7880923748016357,0.7880923748016357,0.7880923748016357,0.7880923748016357,0.7880923748016357,0.7880923748016357,0.7880923748016357,0.7880923748016357,0.8609709739685059,0.8726780414581299,0.913025975227356,0.9159767031669617,0.9295017719268799,0.9395757913589478,0.9528202414512634,0.952684760093689,0.9425256252288818,0.9404691457748413,0.29394790530204773,0.2927365005016327,0.2800493836402893,0.25150224566459656,0.23013834655284882,0.21105656027793884,0.20120839774608612,0.21068733930587769,0.21177330613136292,0.2040606439113617,0.8771056532859802,0.8848785161972046,0.876003086566925,0.8715884685516357,0.8968847393989563,0.8796259164810181,0.863228440284729,0.8937824964523315,0.9055226445198059,0.8790139555931091,-0.4473212957382202,-0.39824146032333374,-0.3454076051712036,-0.296630859375,-0.24432560801506042,-0.19454677402973175,-0.14328306913375854,-0.09466363489627838,-0.046929702162742615,-0.0,0.03610175848007202,0.030106335878372192,0.017831340432167053,0.011607646942138672,-0.0006918162107467651,-0.005270376801490784,-0.010957334190607071,-0.009689223021268845,-0.0066405292600393295,0.0,0.9210164546966553,0.937416136264801,0.9515315294265747,0.964393675327301,0.9751874208450317,0.9841181635856628,0.9909756183624268,0.9959688186645508,0.999005138874054,1.0,-0.38952362537384033,-0.3482111096382141,-0.3075512647628784,-0.26447078585624695,-0.22138097882270813,-0.17751459777355194,-0.1340424120426178,-0.0897001326084137,-0.04459451511502266,0.0};
    float out[70];
    model.evaluate(in, out);

    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 7; j++) {
            cout << setw(4) << out[j*10+i] << " ";
        }cout << endl;
    }
}