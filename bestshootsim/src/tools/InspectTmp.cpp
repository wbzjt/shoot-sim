#include <iostream>
#include <iomanip>
#include <cmath>
#include "model/Search.h"
#include "util/MathUtil.h"
int main(){
  shootersim::SimulationConfig config;
  auto result = shootersim::model::SolveForDistance(config, 1.94);
  std::cout<<std::fixed<<std::setprecision(6);
  std::cout<<"best="<<shootersim::util::RadToDeg(result.bestThetaRad)<<" dv="<<result.bestDeltaV<<" low="<<result.bestVLow<<" high="<<result.bestVHigh<<" nom="<<result.bestVNominal<<"\n";
  for (const auto& scan : result.thetaScan) {
    double deg = shootersim::util::RadToDeg(scan.thetaRad);
    if (!scan.hasValidInterval) continue;
    if ((deg >= 49.5 && deg <= 55.5) || std::fabs(deg - 70.53) < 0.02 || std::fabs(deg - 70.00) < 0.02) {
      std::cout<<deg<<" dv="<<scan.bestInterval.deltaV<<" low="<<scan.bestInterval.vLow<<" high="<<scan.bestInterval.vHigh<<" nom="<<scan.vNominal<<"\n";
    }
  }
}
