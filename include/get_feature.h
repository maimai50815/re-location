#ifndef GET_FEATURE_H
#define GET_FEATURE_H
#include "common.h"

using namespace std;

namespace re_location
{
class GetFeature
{
public:
  GetFeature();
  void setFeatureFromCloud(float&, float&, NormalPtr&);
  void searchByFeature(NormalPtr&, std::vector<PoseInfo>&);
private:
  void getFeature();

};
}/* end of namespace */
#endif
