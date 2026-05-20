from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _extrude_xz_profile(profile: list[tuple[float, float]], half_width_y: float) -> MeshGeometry:
    """Build a simple prism by extruding an X/Z polygon symmetrically along Y."""

    geom = MeshGeometry()
    front: list[int] = []
    back: list[int] = []
    for x, z in profile:
        front.append(geom.add_vertex(x, -half_width_y, z))
    for x, z in profile:
        back.append(geom.add_vertex(x, half_width_y, z))

    # Front and rear caps.
    for i in range(1, len(profile) - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])

    # Side walls.
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="print_shop_guillotine")

    mat_dark = Material("dark cast metal", rgba=(0.07, 0.08, 0.09, 1.0))
    mat_steel = Material("brushed steel", rgba=(0.63, 0.65, 0.64, 1.0))
    mat_edge = Material("sharpened steel edge", rgba=(0.86, 0.88, 0.86, 1.0))
    mat_green = Material("green cutting mat", rgba=(0.05, 0.30, 0.16, 1.0))
    mat_grid = Material("printed grid marks", rgba=(0.82, 0.90, 0.74, 1.0))
    mat_rubber = Material("black rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    mat_red = Material("red lock handle", rgba=(0.70, 0.05, 0.035, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.82, 0.48, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=mat_dark,
        name="base_slab",
    )
    base.visual(
        Box((0.72, 0.38, 0.006)),
        origin=Origin(xyz=(0.02, 0.02, 0.058)),
        material=mat_green,
        name="cutting_mat",
    )

    for index, x in enumerate((-0.30, -0.20, -0.10, 0.00, 0.10, 0.20, 0.30)):
        base.visual(
            Box((0.002, 0.38, 0.002)),
            origin=Origin(xyz=(x, 0.02, 0.0615)),
            material=mat_grid,
            name=f"grid_x_{index}",
        )
    for index, y in enumerate((-0.14, -0.04, 0.06, 0.16)):
        base.visual(
            Box((0.72, 0.002, 0.002)),
            origin=Origin(xyz=(0.02, y, 0.0615)),
            material=mat_grid,
            name=f"grid_y_{index}",
        )

    base.visual(
        Box((0.70, 0.028, 0.008)),
        origin=Origin(xyz=(0.02, -0.205, 0.059)),
        material=mat_edge,
        name="cutting_strip",
    )
    base.visual(
        Box((0.78, 0.024, 0.042)),
        origin=Origin(xyz=(0.0, 0.228, 0.076)),
        material=mat_steel,
        name="rear_fence",
    )
    base.visual(
        Box((0.76, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, -0.235, 0.068)),
        material=mat_steel,
        name="front_guard",
    )

    # A rigid side hinge block: pedestal plus two cheek plates straddling the
    # blade barrel, with visible pin caps on the outside faces.
    for name, y in (("front_hinge_foot", -0.272), ("rear_hinge_foot", -0.138)):
        base.visual(
            Box((0.150, 0.034, 0.056)),
            origin=Origin(xyz=(-0.335, y, 0.083)),
            material=mat_dark,
            name=name,
        )
    base.visual(
        Box((0.150, 0.030, 0.030)),
        origin=Origin(xyz=(-0.335, -0.240, 0.070)),
        material=mat_dark,
        name="front_hinge_bridge",
    )
    base.visual(
        Box((0.080, 0.034, 0.130)),
        origin=Origin(xyz=(-0.335, -0.272, 0.176)),
        material=mat_dark,
        name="front_hinge_cheek",
    )
    base.visual(
        Box((0.080, 0.034, 0.130)),
        origin=Origin(xyz=(-0.335, -0.138, 0.176)),
        material=mat_dark,
        name="rear_hinge_cheek",
    )
    for name, y in (("front_pin_cap", -0.278), ("rear_pin_cap", -0.132)):
        base.visual(
            Cylinder(radius=0.024, length=0.034),
            origin=Origin(xyz=(-0.335, y, 0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mat_steel,
            name=name,
        )

    base.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(-0.230, -0.120, 0.067)),
        material=mat_dark,
        name="lock_boss",
    )

    blade_arm = model.part("blade_arm")
    blade_profile = [
        (0.015, -0.040),
        (0.090, -0.075),
        (0.690, -0.080),
        (0.690, -0.020),
        (0.250, 0.005),
        (0.015, 0.020),
    ]
    blade_arm.visual(
        mesh_from_geometry(_extrude_xz_profile(blade_profile, 0.018), "blade_plate"),
        material=mat_edge,
        name="blade_plate",
    )
    blade_arm.visual(
        Box((0.620, 0.050, 0.024)),
        origin=Origin(xyz=(0.380, 0.0, 0.002)),
        material=mat_steel,
        name="upper_spine",
    )
    blade_arm.visual(
        Cylinder(radius=0.034, length=0.100),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mat_steel,
        name="hinge_barrel",
    )
    blade_arm.visual(
        Box((0.034, 0.034, 0.080)),
        origin=Origin(xyz=(0.640, 0.0, 0.052)),
        material=mat_steel,
        name="handle_post",
    )
    blade_arm.visual(
        Cylinder(radius=0.026, length=0.160),
        origin=Origin(xyz=(0.670, 0.0, 0.102), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mat_rubber,
        name="handle_grip",
    )

    lock_handle = model.part("lock_handle")
    lock_handle.visual(
        Cylinder(radius=0.011, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=mat_steel,
        name="lock_shaft",
    )
    lock_handle.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=mat_red,
        name="round_handle",
    )
    lock_handle.visual(
        Box((0.085, 0.014, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, 0.075)),
        material=mat_rubber,
        name="lock_pointer",
    )

    model.articulation(
        "base_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(-0.335, -0.205, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "base_to_lock_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lock_handle,
        origin=Origin(xyz=(-0.230, -0.120, 0.073)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    lock_handle = object_model.get_part("lock_handle")
    blade_joint = object_model.get_articulation("base_to_blade_arm")
    lock_joint = object_model.get_articulation("base_to_lock_handle")

    ctx.expect_gap(
        blade_arm,
        base,
        axis="z",
        positive_elem="blade_plate",
        negative_elem="cutting_strip",
        min_gap=0.004,
        max_gap=0.020,
        name="closed blade rides just above the cutting strip",
    )
    ctx.expect_overlap(
        blade_arm,
        base,
        axes="xy",
        elem_a="blade_plate",
        elem_b="cutting_strip",
        min_overlap=0.015,
        name="blade edge tracks over the side cutting strip",
    )
    ctx.expect_contact(
        lock_handle,
        base,
        elem_a="lock_shaft",
        elem_b="lock_boss",
        contact_tol=0.0015,
        name="lock handle shaft seats on the hinge-block boss",
    )

    closed_aabb = ctx.part_element_world_aabb(blade_arm, elem="handle_grip")
    with ctx.pose({blade_joint: 1.25}):
        open_aabb = ctx.part_element_world_aabb(blade_arm, elem="handle_grip")
    ctx.check(
        "blade arm opens upward about the side hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.35,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    pointer_closed = ctx.part_element_world_aabb(lock_handle, elem="lock_pointer")
    with ctx.pose({lock_joint: math.pi / 2.0}):
        pointer_rotated = ctx.part_element_world_aabb(lock_handle, elem="lock_pointer")
    if pointer_closed is not None and pointer_rotated is not None:
        closed_dx = pointer_closed[1][0] - pointer_closed[0][0]
        closed_dy = pointer_closed[1][1] - pointer_closed[0][1]
        rotated_dx = pointer_rotated[1][0] - pointer_rotated[0][0]
        rotated_dy = pointer_rotated[1][1] - pointer_rotated[0][1]
        lock_rotates = closed_dx > closed_dy * 2.5 and rotated_dy > rotated_dx * 2.5
    else:
        lock_rotates = False
    ctx.check(
        "rotary lock handle turns on its own vertical shaft",
        lock_rotates,
        details=f"closed={pointer_closed}, rotated={pointer_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
