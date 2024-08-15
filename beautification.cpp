#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3> Point_with_normal;

std::vector<Point_with_normal> read_points_from_csv(const std::string& filename) {
    std::vector<Point_with_normal> points;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream stream(line);
        double x, y, z, nx, ny, nz;
        char delimiter;
        stream >> x >> delimiter >> y >> delimiter >> z >> delimiter >> nx >> delimiter >> ny >> delimiter >> nz;

        points.emplace_back(Kernel::Point_3(x, y, z), Kernel::Vector_3(nx, ny, nz));
    }
    return points;
}

void write_shapes_to_csv(const std::string& filename, const Efficient_ransac::Shape_range& shapes) {
    std::ofstream file(filename);
    file << "Shape,Parameters,Average Distance" << std::endl;

    for (auto it = shapes.begin(); it != shapes.end(); ++it) {
        if (Plane* plane = dynamic_cast<Plane*>(it->get())) {
            Kernel::Vector_3 normal = plane->plane_normal();
            file << "Plane" << "," << normal << ",";

        } else if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get())) {
            Kernel::Line_3 axis = cyl->axis();
            double radius = cyl->radius();
            file << "Cylinder" << "," << "Axis: " << axis << ", Radius: " << radius << ",";

        } else {
            file << "Other Shape" << "," << (*it)->info() << ",";
        }

        double sum_distances = 0;
        for (auto index : (*it)->indices_of_assigned_points()) {
            const Point_with_normal& p = points[index];
            sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));
        }
        double average_distance = sum_distances / (*it)->indices_of_assigned_points().size();
        file << average_distance << std::endl;
    }
}

int main(int argc, char** argv) {
    // Points with normals.
    Pwn_vector points;

    // Load point set from a CSV file.
    if (argc > 1) {
        points = read_points_from_csv(argv[1]);
    } else {
        std::cerr << "Please provide a CSV file as input!" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << points.size() << " points loaded from CSV." << std::endl;

    // Instantiate shape detection engine.
    Efficient_ransac ransac;

    // Provide input data.
    ransac.set_input(points);

    // Register shapes for detection.
    ransac.add_shape_factory<Plane>();
    ransac.add_shape_factory<Sphere>();
    ransac.add_shape_factory<Cylinder>();
    ransac.add_shape_factory<Cone>();
    ransac.add_shape_factory<Torus>();

    // Set parameters for shape detection.
    Efficient_ransac::Parameters parameters;
    parameters.probability = 0.05;
    parameters.min_points = 200;
    parameters.epsilon = 0.002;
    parameters.cluster_epsilon = 0.01;
    parameters.normal_threshold = 0.9;

    // Detect shapes.
    ransac.detect(parameters);

    // Print number of detected shapes and unassigned points.
    std::cout << ransac.shapes().end() - ransac.shapes().begin()
              << " detected shapes, "
              << ransac.number_of_unassigned_points()
              << " unassigned points." << std::endl;

    // Write detected shapes to a CSV file.
    if (argc > 2) {
        write_shapes_to_csv(argv[2], ransac.shapes());
        std::cout << "Detected shapes written to CSV file: " << argv[2] << std::endl;
    } else {
        std::cerr << "Please provide an output CSV file path as the second argument!" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
