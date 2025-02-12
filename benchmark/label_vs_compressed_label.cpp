#include <array>
#include <vector>

/* #include "../datastructures/bit_vector.h" */
/* #include "../datastructures/golomb_rice.h" */
#include "../datastructures/hub_labels.h"
#include "../external/cmdparser.hpp"

void configure_parser(cli::Parser& parser) {
  parser.set_required<std::string>("i", "input_labels", "Input label file.");
};

int main(int argc, char* argv[]) {
  cli::Parser parser(argc, argv);
  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");

  std::size_t mem_raw_bytes = 0;
  std::size_t mem_compressed_bytes = 0;
  std::size_t mem_delta_compressed_bytes = 0;
  std::size_t mem_simple_compressed_bytes = 0;
  /* std::size_t mem_simple_golomb_rice_bytes = 0; */
  /* std::size_t mem_delta_golomb_rice_bytes = 0; */

  std::array<std::vector<Label>, 2> labels;

  readFromFile(labels, inputFileName);

  showStats(labels);

  /* runQueries(labels); */

  mem_raw_bytes = computeTotalBytes(labels);

  {
    std::array<std::vector<CompressedLabel>, 2> comp_labels;

    comp_labels[FWD].reserve(labels[FWD].size());
    comp_labels[BWD].reserve(labels[BWD].size());

    for (std::size_t v = 0; v < labels[FWD].size(); ++v) {
      comp_labels[FWD].emplace_back(labels[FWD][v]);
      comp_labels[BWD].emplace_back(labels[BWD][v]);
    }

    for (std::size_t v = 0; v < comp_labels[FWD].size(); ++v) {
      mem_compressed_bytes += comp_labels[FWD][v].byteSize();
      mem_compressed_bytes += comp_labels[BWD][v].byteSize();
    }
  }
  {
    std::array<std::vector<DeltaCompressedLabel>, 2> comp_labels;

    comp_labels[FWD].reserve(labels[FWD].size());
    comp_labels[BWD].reserve(labels[BWD].size());

    for (std::size_t v = 0; v < labels[FWD].size(); ++v) {
      comp_labels[FWD].emplace_back(labels[FWD][v]);
      comp_labels[BWD].emplace_back(labels[BWD][v]);
    }

    for (std::size_t v = 0; v < comp_labels[FWD].size(); ++v) {
      mem_delta_compressed_bytes += comp_labels[FWD][v].byteSize();
      mem_delta_compressed_bytes += comp_labels[BWD][v].byteSize();
    }
  }

  {
    std::array<std::vector<SimpleCompressedLabel>, 2> comp_labels;

    comp_labels[FWD].reserve(labels[FWD].size());
    comp_labels[BWD].reserve(labels[BWD].size());

    for (std::size_t v = 0; v < labels[FWD].size(); ++v) {
      comp_labels[FWD].emplace_back(labels[FWD][v]);
      comp_labels[BWD].emplace_back(labels[BWD][v]);
    }

    for (std::size_t v = 0; v < comp_labels[FWD].size(); ++v) {
      mem_simple_compressed_bytes += comp_labels[FWD][v].byteSize();
      mem_simple_compressed_bytes += comp_labels[BWD][v].byteSize();
    }
  }
  /* { */
  /*   GolombRice<uint32_t, uint8_t> coder(128); */

  /*   std::array<std::vector<BitVector<uint8_t>>, 2> bit_vectors; */

  /*   bit_vectors[FWD].reserve(labels[FWD].size()); */
  /*   bit_vectors[BWD].reserve(labels[BWD].size()); */

  /*   auto toDelta = [](const std::vector<uint32_t>& from, */
  /*                     std::vector<uint32_t>& to) -> void { */
  /*     to.clear(); */

  /*     to.push_back(from.front()); */

  /*     for (size_t i = 1; i < from.size(); ++i) { */
  /*       to.push_back(from[i] - from[i - 1] - 1); */
  /*     } */
  /*   }; */

  /*   for (std::size_t v = 0; v < labels[FWD].size(); ++v) { */
  /*     std::vector<uint32_t> delta; */

  /*     toDelta(labels[FWD][v].nodes, delta); */
  /*     bit_vectors[FWD].emplace_back(coder.encode(delta)); */

  /*     toDelta(labels[BWD][v].nodes, delta); */
  /*     bit_vectors[BWD].emplace_back(coder.encode(delta)); */
  /*   } */

  /*   for (std::size_t v = 0; v < bit_vectors[FWD].size(); ++v) { */
  /*     mem_delta_golomb_rice_bytes += bit_vectors[FWD][v].byteSize(); */
  /*     mem_delta_golomb_rice_bytes += bit_vectors[BWD][v].byteSize(); */
  /*   } */
  /* } */

  /* { */
  /*   GolombRice<uint32_t, uint8_t> coder(128); */

  /*   std::array<std::vector<BitVector<uint8_t>>, 2> bit_vectors; */

  /*   bit_vectors[FWD].reserve(labels[FWD].size()); */
  /*   bit_vectors[BWD].reserve(labels[BWD].size()); */

  /*   for (std::size_t v = 0; v < labels[FWD].size(); ++v) { */
  /*     bit_vectors[FWD].emplace_back(coder.encode(labels[FWD][v].nodes)); */
  /*     bit_vectors[BWD].emplace_back(coder.encode(labels[BWD][v].nodes)); */
  /*   } */

  /*   for (std::size_t v = 0; v < bit_vectors[FWD].size(); ++v) { */
  /*     mem_simple_golomb_rice_bytes += bit_vectors[FWD][v].byteSize(); */
  /*     mem_simple_golomb_rice_bytes += bit_vectors[BWD][v].byteSize(); */
  /*   } */
  /* } */

  std::cout << "Memory [mb]:        " << std::endl;
  std::cout << "Raw:                " << (mem_raw_bytes / (1024 * 1024))
            << std::endl;
  std::cout << "Compressed:         " << (mem_compressed_bytes / (1024 * 1024))
            << std::endl;
  std::cout << "Delta Compressed:   "
            << (mem_delta_compressed_bytes / (1024 * 1024)) << std::endl;
  std::cout << "Simple Compressed:  "
            << (mem_simple_compressed_bytes / (1024 * 1024)) << std::endl;
  /* std::cout << "Delta Golomb Rice: " */
  /*           << (mem_delta_golomb_rice_bytes / (1024 * 1024)) << std::endl; */
  /* std::cout << "Simple Golomb Rice: " */
  /*           << (mem_simple_golomb_rice_bytes / (1024 * 1024)) << std::endl;
   */

  return 0;
}

/*
# this is without -c (compress)
>>> ./LabelVSCompLabel -i ../data/icice.labels
Forward Labels Statistics:
  Min Size:     1
  Max Size:     222
  Avg Size:     64.827
Backward Labels Statistics:
  Min Size:     1
  Max Size:     133
  Avg Size:     42.7677
FWD # count:    12062224
BWD # count:    7957702
Both # count:   20019926
Total memory consumption [megabytes]:
  117.435
Memory [mb]:
Raw:                117
Compressed:         78
Delta Compressed:   56
Simple Compressed:  125

>>> ./LabelVSCompLabel -i ../data/kvv.labels
Forward Labels Statistics:
  Min Size:     1
  Max Size:     240
  Avg Size:     78.559
Backward Labels Statistics:
  Min Size:     1
  Max Size:     227
  Avg Size:     69.6258
FWD # count:    198549287
BWD # count:    175971501
Both # count:   374520788
Total memory consumption [megabytes]:
  2149.54
Memory [mb]:
Raw:                2149
Compressed:         1431
Delta Compressed:   999
Simple Compressed:  2265

# this is with -c (compress)
>>> ./LabelVSCompLabel -i ../data/icice.labels
Forward Labels Statistics:
  Min Size:     1
  Max Size:     222
  Avg Size:     64.8339
Backward Labels Statistics:
  Min Size:     1
  Max Size:     133
  Avg Size:     42.7785
FWD # count:    12063512
BWD # count:    7959709
Both # count:   20023221
Total memory consumption [megabytes]:
  117.445
Memory [mb]:
Raw:                117
Compressed:         58
Delta Compressed:   49
Simple Compressed:  103

>>> ./LabelVSCompLabel -i ../data/kvv.labels
Forward Labels Statistics:
  Min Size:     1
  Max Size:     240
  Avg Size:     78.5597
Backward Labels Statistics:
  Min Size:     1
  Max Size:     227
  Avg Size:     69.6284
FWD # count:    198550988
BWD # count:    175978093
Both # count:   374529081
Total memory consumption [megabytes]:
  2149.59
Memory [mb]:
Raw:                2149
Compressed:         1130
Delta Compressed:   950
Simple Compressed:  2089
*/
