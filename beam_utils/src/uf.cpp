#include "beam_utils/uf.hpp"

namespace beam {

void UF::Initialize(int n) {
  for (int i = 0; i < n; i++) {
    id_.push_back(i);
    rank_.push_back(0);
  }
}

int UF::FindSet(int p) {
  if (p != id_[p]) { id_[p] = this->FindSet(id_[p]); }
}

void UF::UnionSets(int p, int q) {
  int i = this->FindSet(p);
  int j = this->FindSet(q);
  if (i != j) {
    if (rank_[i] < rank_[j]) {
      id_[i] = j;
    } else if (rank_[i] > rank_[j]) {
      id_[j] = i;
    } else {
      id_[j] = i;
      rank_[i] += 1;
    }
  }
}

} // namespace beam