import numpy as np
import g2o


def add_vertex(optimizer, id, pose, fixed=False):
    v_se3 = g2o.VertexSE3()
    v_se3.set_id(id)
    v_se3.set_estimate(pose)
    v_se3.set_fixed(fixed)
    optimizer.add_vertex(v_se3)
    return optimizer

def add_edge(optimizer, vertices, measurement,
             information=np.identity(6),
             robust_kernel=None):

    edge = g2o.EdgeSE3()
    for i, v in enumerate(vertices):
        if isinstance(v, int):
            v = optimizer.vertex(v)
        edge.set_vertex(i, v)

    edge.set_measurement(measurement)  # relative pose
    edge.set_information(information)
    if robust_kernel is not None:
        edge.set_robust_kernel(robust_kernel)
    optimizer.add_edge(edge)
    return optimizer

def main():
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(solver)
    poseArr = np.array([[2, 5, 5], [2, 2, 6], [0, 0, 0]])
    for i in range(3):
        pose = g2o.Isometry3d(np.identity(3), poseArr[i])
        optimizer = add_vertex(optimizer, i, pose)
    optimizer = add_edge(optimizer, [0, 3], g2o.Isometry3d(np.identity(3), [4, 0, 0]))
    optimizer = add_edge(optimizer, [1, 3], g2o.Isometry3d(np.identity(3), [5, 0, 0]))
    optimizer = add_edge(optimizer, [2, 3], g2o.Isometry3d(np.identity(3), [3, 0, 0]))
    optimizer = add_edge(optimizer, [0, 1], g2o.Isometry3d(np.identity(3), [3, 0, 0]))
    optimizer = add_edge(optimizer, [0, 2], g2o.Isometry3d(np.identity(3), [5, 0, 0]))
    optimizer = add_edge(optimizer, [1, 2], g2o.Isometry3d(np.identity(3), [4, 0, 0]))

    optimizer.initialize_optimization()
    optimizer.set_verbose(True)
    optimizer.optimize(3)
    #print(int(optimizer.vertex(0).estimate())

if __name__ == "__main__":
    main()
