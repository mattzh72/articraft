from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _normalize(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2)
    if length <= 1e-9:
        return (1.0, 0.0, 0.0)
    return (vec[0] / length, vec[1] / length, vec[2] / length)


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _curved_hollow_tube_mesh(
    path_points: list[tuple[float, float, float]],
    outer_radii: list[float],
    inner_radii: list[float],
    *,
    radial_segments: int = 40,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer_rings: list[list[int]] = []
    inner_rings: list[list[int]] = []
    y_axis = (0.0, 1.0, 0.0)

    for index, point in enumerate(path_points):
        prev_pt = path_points[index - 1] if index > 0 else path_points[index]
        next_pt = path_points[index + 1] if index < len(path_points) - 1 else path_points[index]
        tangent = _normalize(
            (
                next_pt[0] - prev_pt[0],
                next_pt[1] - prev_pt[1],
                next_pt[2] - prev_pt[2],
            )
        )
        xz_normal = _normalize(_cross(y_axis, tangent))
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for seg in range(radial_segments):
            angle = 2.0 * math.pi * seg / radial_segments
            c = math.cos(angle)
            s = math.sin(angle)
            direction = (
                y_axis[0] * c + xz_normal[0] * s,
                y_axis[1] * c + xz_normal[1] * s,
                y_axis[2] * c + xz_normal[2] * s,
            )
            outer_ring.append(
                geom.add_vertex(
                    point[0] + direction[0] * outer_radii[index],
                    point[1] + direction[1] * outer_radii[index],
                    point[2] + direction[2] * outer_radii[index],
                )
            )
            inner_ring.append(
                geom.add_vertex(
                    point[0] + direction[0] * inner_radii[index],
                    point[1] + direction[1] * inner_radii[index],
                    point[2] + direction[2] * inner_radii[index],
                )
            )
        outer_rings.append(outer_ring)
        inner_rings.append(inner_ring)

    for ring_index in range(len(path_points) - 1):
        outer_a = outer_rings[ring_index]
        outer_b = outer_rings[ring_index + 1]
        inner_a = inner_rings[ring_index]
        inner_b = inner_rings[ring_index + 1]
        for seg in range(radial_segments):
            nxt = (seg + 1) % radial_segments

            geom.add_face(outer_a[seg], outer_a[nxt], outer_b[nxt])
            geom.add_face(outer_a[seg], outer_b[nxt], outer_b[seg])

            geom.add_face(inner_a[seg], inner_b[seg], inner_b[nxt])
            geom.add_face(inner_a[seg], inner_b[nxt], inner_a[nxt])

    start_outer = outer_rings[0]
    start_inner = inner_rings[0]
    end_outer = outer_rings[-1]
    end_inner = inner_rings[-1]
    for seg in range(radial_segments):
        nxt = (seg + 1) % radial_segments

        geom.add_face(start_outer[seg], start_inner[nxt], start_outer[nxt])
        geom.add_face(start_outer[seg], start_inner[seg], start_inner[nxt])

        geom.add_face(end_outer[seg], end_outer[nxt], end_inner[nxt])
        geom.add_face(end_outer[seg], end_inner[nxt], end_inner[seg])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="de_dion_rear_axle")

    chassis_black = model.material("chassis_black", rgba=(0.13, 0.13, 0.14, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.35, 0.38, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    chassis_mount = model.part("chassis_mount")
    chassis_mount.visual(
        Box((1.02, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=chassis_black,
        name="elem_crossmember",
    )
    chassis_mount.visual(
        Box((0.54, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=painted_steel,
        name="elem_subframe_bridge",
    )
    chassis_mount.visual(
        Sphere(radius=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.304)),
        material=dark_steel,
        name="elem_differential_housing",
    )
    chassis_mount.visual(
        Cylinder(radius=0.040, length=0.11),
        origin=Origin(xyz=(0.0, 0.072, 0.302), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elem_pinion_nose",
    )
    chassis_mount.visual(
        Box((0.08, 0.06, 0.16)),
        origin=Origin(xyz=(-0.28, 0.0, 0.29)),
        material=painted_steel,
        name="elem_left_drop_link",
    )
    chassis_mount.visual(
        Box((0.08, 0.06, 0.16)),
        origin=Origin(xyz=(0.28, 0.0, 0.29)),
        material=painted_steel,
        name="elem_right_drop_link",
    )
    chassis_mount.inertial = Inertial.from_geometry(
        Box((1.10, 0.24, 0.42)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )

    de_dion_tube = model.part("de_dion_tube")
    tube_path = [
        (-0.78, 0.0, 0.130),
        (-0.66, 0.0, 0.130),
        (-0.48, 0.0, 0.132),
        (-0.24, 0.0, 0.138),
        (0.00, 0.0, 0.143),
        (0.24, 0.0, 0.138),
        (0.48, 0.0, 0.132),
        (0.66, 0.0, 0.130),
        (0.78, 0.0, 0.130),
    ]
    tube_outer = [0.073, 0.066, 0.052, 0.050, 0.051, 0.050, 0.052, 0.066, 0.073]
    tube_inner = [0.043, 0.041, 0.040, 0.040, 0.040, 0.040, 0.040, 0.041, 0.043]
    axle_shell = mesh_from_geometry(
        _curved_hollow_tube_mesh(tube_path, tube_outer, tube_inner, radial_segments=42),
        "de_dion_axle_shell",
    )
    de_dion_tube.visual(
        axle_shell,
        material=painted_steel,
        name="elem_axle_shell",
    )
    de_dion_tube.visual(
        Box((0.09, 0.07, 0.028)),
        origin=Origin(xyz=(-0.28, 0.0, 0.196)),
        material=painted_steel,
        name="elem_left_saddle",
    )
    de_dion_tube.visual(
        Box((0.09, 0.07, 0.028)),
        origin=Origin(xyz=(0.28, 0.0, 0.196)),
        material=painted_steel,
        name="elem_right_saddle",
    )
    de_dion_tube.visual(
        Box((0.16, 0.08, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        material=painted_steel,
        name="elem_center_bridge",
    )
    de_dion_tube.inertial = Inertial.from_geometry(
        Box((1.66, 0.18, 0.16)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )

    left_half_shaft = model.part("left_half_shaft")
    left_half_shaft.visual(
        Sphere(radius=0.024),
        material=rubber,
        name="elem_inner_cv",
    )
    left_half_shaft.visual(
        Cylinder(radius=0.021, length=0.060),
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="elem_inner_tulip",
    )
    left_half_shaft.visual(
        Cylinder(radius=0.014, length=0.600),
        origin=Origin(xyz=(-0.260, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="elem_shaft_core",
    )
    left_half_shaft.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(-0.520, 0.0, 0.0)),
        material=rubber,
        name="elem_outer_cv",
    )
    left_half_shaft.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(-0.550, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="elem_outer_tulip",
    )
    left_half_shaft.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(-0.610, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="elem_outer_stub",
    )
    left_half_shaft.inertial = Inertial.from_geometry(
        Box((0.77, 0.07, 0.07)),
        mass=8.0,
        origin=Origin(xyz=(-0.36, 0.0, 0.0)),
    )

    right_half_shaft = model.part("right_half_shaft")
    right_half_shaft.visual(
        Sphere(radius=0.024),
        material=rubber,
        name="elem_inner_cv",
    )
    right_half_shaft.visual(
        Cylinder(radius=0.021, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="elem_inner_tulip",
    )
    right_half_shaft.visual(
        Cylinder(radius=0.014, length=0.600),
        origin=Origin(xyz=(0.260, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="elem_shaft_core",
    )
    right_half_shaft.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.520, 0.0, 0.0)),
        material=rubber,
        name="elem_outer_cv",
    )
    right_half_shaft.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.550, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="elem_outer_tulip",
    )
    right_half_shaft.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(0.610, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="elem_outer_stub",
    )
    right_half_shaft.inertial = Inertial.from_geometry(
        Box((0.77, 0.07, 0.07)),
        mass=8.0,
        origin=Origin(xyz=(0.36, 0.0, 0.0)),
    )

    left_hub = model.part("left_hub")
    left_hub.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(-0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="elem_hub_journal",
    )
    left_hub.visual(
        Cylinder(radius=0.062, length=0.016),
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="elem_bearing_land",
    )
    left_hub.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(-0.059, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="elem_hub_barrel",
    )
    left_hub.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(xyz=(-0.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="elem_hub_flange",
    )
    left_hub.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(-0.096, 0.0, 0.0)),
        material=dark_steel,
        name="elem_hub_cap",
    )
    left_hub.inertial = Inertial.from_geometry(
        Box((0.13, 0.17, 0.17)),
        mass=6.0,
        origin=Origin(xyz=(-0.050, 0.0, 0.0)),
    )

    right_hub = model.part("right_hub")
    right_hub.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="elem_hub_journal",
    )
    right_hub.visual(
        Cylinder(radius=0.062, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="elem_bearing_land",
    )
    right_hub.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.059, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="elem_hub_barrel",
    )
    right_hub.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="elem_hub_flange",
    )
    right_hub.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.096, 0.0, 0.0)),
        material=dark_steel,
        name="elem_hub_cap",
    )
    right_hub.inertial = Inertial.from_geometry(
        Box((0.13, 0.17, 0.17)),
        mass=6.0,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
    )

    model.articulation(
        "chassis_to_axle",
        ArticulationType.FIXED,
        parent=chassis_mount,
        child=de_dion_tube,
        origin=Origin(),
    )
    model.articulation(
        "left_half_shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=de_dion_tube,
        child=left_half_shaft,
        origin=Origin(xyz=(-0.140, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=60.0),
    )
    model.articulation(
        "right_half_shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=de_dion_tube,
        child=right_half_shaft,
        origin=Origin(xyz=(0.140, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=60.0),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=de_dion_tube,
        child=left_hub,
        origin=Origin(xyz=(-0.780, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=80.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=de_dion_tube,
        child=right_hub,
        origin=Origin(xyz=(0.780, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    chassis_mount = object_model.get_part("chassis_mount")
    de_dion_tube = object_model.get_part("de_dion_tube")
    left_half_shaft = object_model.get_part("left_half_shaft")
    right_half_shaft = object_model.get_part("right_half_shaft")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")

    left_half_shaft_spin = object_model.get_articulation("left_half_shaft_spin")
    right_half_shaft_spin = object_model.get_articulation("right_half_shaft_spin")
    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")

    for part_name in (
        "chassis_mount",
        "de_dion_tube",
        "left_half_shaft",
        "right_half_shaft",
        "left_hub",
        "right_hub",
    ):
        ctx.check(f"{part_name} exists", object_model.get_part(part_name) is not None)

    for joint_name in (
        "left_half_shaft_spin",
        "right_half_shaft_spin",
        "left_hub_spin",
        "right_hub_spin",
    ):
        joint = object_model.get_articulation(joint_name)
        axis_ok = (
            joint.articulation_type == ArticulationType.CONTINUOUS
            and abs(abs(joint.axis[0]) - 1.0) < 1e-6
            and abs(joint.axis[1]) < 1e-6
            and abs(joint.axis[2]) < 1e-6
        )
        ctx.check(
            f"{joint_name} is continuous about axle axis",
            axis_ok,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_contact(
        chassis_mount,
        de_dion_tube,
        elem_a="elem_left_drop_link",
        elem_b="elem_left_saddle",
        name="left axle saddle is clamped to the chassis link",
    )
    ctx.expect_contact(
        chassis_mount,
        de_dion_tube,
        elem_a="elem_right_drop_link",
        elem_b="elem_right_saddle",
        name="right axle saddle is clamped to the chassis link",
    )

    ctx.expect_within(
        left_half_shaft,
        de_dion_tube,
        axes="yz",
        inner_elem="elem_shaft_core",
        outer_elem="elem_axle_shell",
        margin=0.001,
        name="left shaft stays inside the de Dion tube laterally and vertically",
    )
    ctx.expect_within(
        right_half_shaft,
        de_dion_tube,
        axes="yz",
        inner_elem="elem_shaft_core",
        outer_elem="elem_axle_shell",
        margin=0.001,
        name="right shaft stays inside the de Dion tube laterally and vertically",
    )
    ctx.expect_overlap(
        left_half_shaft,
        de_dion_tube,
        axes="x",
        elem_a="elem_shaft_core",
        elem_b="elem_axle_shell",
        min_overlap=0.55,
        name="left shaft remains retained along the tube length",
    )
    ctx.expect_overlap(
        right_half_shaft,
        de_dion_tube,
        axes="x",
        elem_a="elem_shaft_core",
        elem_b="elem_axle_shell",
        min_overlap=0.55,
        name="right shaft remains retained along the tube length",
    )

    ctx.expect_origin_distance(
        left_hub,
        left_half_shaft,
        axes="yz",
        min_dist=0.0,
        max_dist=0.001,
        name="left hub stays coaxial with the left half-shaft",
    )
    ctx.expect_origin_distance(
        right_hub,
        right_half_shaft,
        axes="yz",
        min_dist=0.0,
        max_dist=0.001,
        name="right hub stays coaxial with the right half-shaft",
    )
    ctx.expect_overlap(
        left_hub,
        de_dion_tube,
        axes="yz",
        elem_a="elem_hub_journal",
        elem_b="elem_axle_shell",
        min_overlap=0.05,
        name="left hub journal is aligned with the tube-end carrier",
    )
    ctx.expect_contact(
        left_hub,
        de_dion_tube,
        elem_a="elem_bearing_land",
        elem_b="elem_axle_shell",
        name="left hub bearing land is seated against the tube-end carrier",
    )
    ctx.expect_overlap(
        right_hub,
        de_dion_tube,
        axes="yz",
        elem_a="elem_hub_journal",
        elem_b="elem_axle_shell",
        min_overlap=0.05,
        name="right hub journal is aligned with the tube-end carrier",
    )
    ctx.expect_contact(
        right_hub,
        de_dion_tube,
        elem_a="elem_bearing_land",
        elem_b="elem_axle_shell",
        name="right hub bearing land is seated against the tube-end carrier",
    )

    with ctx.pose(
        {
            left_half_shaft_spin: 1.5,
            right_half_shaft_spin: -1.2,
            left_hub_spin: 0.8,
            right_hub_spin: -0.6,
        }
    ):
        ctx.expect_within(
            left_half_shaft,
            de_dion_tube,
            axes="yz",
            inner_elem="elem_shaft_core",
            outer_elem="elem_axle_shell",
            margin=0.001,
            name="left shaft remains centered in the tube while spinning",
        )
        ctx.expect_within(
            right_half_shaft,
            de_dion_tube,
            axes="yz",
            inner_elem="elem_shaft_core",
            outer_elem="elem_axle_shell",
            margin=0.001,
            name="right shaft remains centered in the tube while spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
