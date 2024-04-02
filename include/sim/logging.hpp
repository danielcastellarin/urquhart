#include <observation.hpp>
#include <geometry.hpp>

namespace logging {

    void writeObservationToFile(std::string filePath, const std::vector<Tree>& trees) {
        std::ofstream out(filePath+".txt");
        for (const Tree& t : trees) out << t.toString() << std::endl;
        out.close();
    }

    void writeHierarchyToFile(const urquhart::Observation& trees, std::ofstream& plyOut, std::ofstream& triOut, std::ofstream& hieOut, std::ofstream& dscOut) {
        trees.hier->viewTree(hieOut);
        
        // Iterate over the indices of the Polygons in the hierarchy
        for(auto pIdx : trees.hier->getChildrenIds(0)) {
            for (int i = 0; i < trees.hier->getPolygon(pIdx).landmarkRefs.size(); ++i) {
                auto myPoint =  trees.landmarks.col(trees.hier->getPolygon(pIdx).landmarkRefs(i));
                plyOut << pIdx << " " << myPoint[0] << " " << myPoint[1] << "|";
            }
            plyOut << std::endl;
            for(auto d : trees.hier->getPolygon(pIdx).descriptor) dscOut << d << " ";
            dscOut << std::endl;
            
            // Iterate over the indices of the Triangles that compose this Polygon
            for(auto tIdx : trees.hier->getChildrenIds(pIdx)) {
                // Retain only the Polygon objects that have three sides
                if (trees.hier->getPolygon(tIdx).n == 3) {
                    for (int i = 0; i < trees.hier->getPolygon(tIdx).landmarkRefs.size(); ++i) {
                        auto myPoint = trees.landmarks.col(trees.hier->getPolygon(tIdx).landmarkRefs(i));
                        triOut << tIdx << " " << myPoint[0] << " " << myPoint[1] << "|";
                    }
                    triOut << std::endl;
                }
            }
        }
    }

}
