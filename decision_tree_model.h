


    // !!! This file is generated using emlearn !!!

    #include <eml_trees.h>
    

static const EmlTreesNode decision_tree_model_nodes[7] = {
  { 99, 0, 1, 4 },
  { 1, 0, 1, 2 },
  { 61, 0, -1, -2 },
  { 60, 0, -2, -1 },
  { 5, 0, 1, 2 },
  { 90, 0, -3, -4 },
  { 47, 1, -3, -4 } 
};

static const int32_t decision_tree_model_tree_roots[1] = { 0 };

static const uint8_t decision_tree_model_leaves[4] = { 0, 3, 1, 2 };

EmlTrees decision_tree_model = {
        7,
        (EmlTreesNode *)(decision_tree_model_nodes),	  
        1,
        (int32_t *)(decision_tree_model_tree_roots),
        4,
        (uint8_t *)(decision_tree_model_leaves),
        0,
        118,
        4,
    };

static inline int32_t decision_tree_model_tree_0(const int16_t *features, int32_t features_length) {
          if (features[99] < 0) {
              if (features[1] < 0) {
                  if (features[61] < 0) {
                      return 0;
                  } else {
                      return 3;
                  }
              } else {
                  if (features[60] < 0) {
                      return 3;
                  } else {
                      return 0;
                  }
              }
          } else {
              if (features[5] < 0) {
                  if (features[90] < 0) {
                      return 1;
                  } else {
                      return 2;
                  }
              } else {
                  if (features[47] < 1) {
                      return 1;
                  } else {
                      return 2;
                  }
              }
          }
        }
        

int32_t decision_tree_model_predict(const int16_t *features, int32_t features_length) {

        int32_t votes[4] = {0,};
        int32_t _class = -1;

        _class = decision_tree_model_tree_0(features, features_length); votes[_class] += 1;
    
        int32_t most_voted_class = -1;
        int32_t most_voted_votes = 0;
        for (int32_t i=0; i<4; i++) {

            if (votes[i] > most_voted_votes) {
                most_voted_class = i;
                most_voted_votes = votes[i];
            }
        }
        return most_voted_class;
    }
    