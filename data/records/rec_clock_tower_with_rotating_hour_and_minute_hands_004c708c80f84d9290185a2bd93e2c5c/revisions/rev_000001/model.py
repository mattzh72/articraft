from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _annulus(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """A centered washer in local XY with its normal on local +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def _marker_ring(radius: float, thickness: float) -> cq.Workplane:
    """Connected raised hour ticks joined by a slim annular track."""
    track = _annulus(radius + 0.012, radius - 0.012, thickness)
    result = track
    for index in range(12):
        angle = index * 30.0
        is_quarter = index % 3 == 0
        tangential = 0.035 if is_quarter else 0.020
        radial = 0.145 if is_quarter else 0.100
        tick_center = radius - radial * 0.28
        tick = (
            cq.Workplane("XY")
            .box(tangential, radial, thickness)
            .translate((0.0, tick_center, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        result = result.union(tick)
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_glass_clock_tower")

    steel = model.material("brushed_steel", rgba=(0.56, 0.59, 0.60, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.05, 0.06, 0.07, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.55, 0.82, 0.95, 0.36))
    frosted_glass = model.material("frosted_clock_glass", rgba=(0.86, 0.94, 0.98, 0.70))
    concrete = model.material("dark_concrete", rgba=(0.22, 0.23, 0.24, 1.0))
    warm_led = model.material("warm_hour_marks", rgba=(0.92, 0.82, 0.56, 1.0))

    dial_ring_mesh = mesh_from_cadquery(
        _annulus(0.675, 0.605, 0.055),
        "beveled_clock_rim",
        tolerance=0.001,
        angular_tolerance=0.08,
    )
    marker_mesh = mesh_from_cadquery(
        _marker_ring(0.490, 0.014),
        "connected_hour_marks",
        tolerance=0.001,
        angular_tolerance=0.08,
    )
    hour_hub_mesh = mesh_from_cadquery(
        _annulus(0.075, 0.030, 0.030),
        "hollow_hour_hub",
        tolerance=0.001,
        angular_tolerance=0.08,
    )

    tower = model.part("tower")

    # Real-world scale: a compact urban tower roughly ten meters tall, with a
    # square glazed shaft and a larger clock head.
    base_size = 2.45
    shaft_size = 1.18
    shaft_height = 7.45
    head_size = 1.86
    head_height = 1.82
    shaft_center_z = 0.36 + shaft_height / 2.0
    head_center_z = 0.36 + shaft_height + head_height / 2.0 - 0.05
    face_center_z = head_center_z

    tower.visual(
        Box((base_size, base_size, 0.36)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=concrete,
        name="granite_plinth",
    )
    tower.visual(
        Box((1.95, 1.95, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=steel,
        name="steel_sill",
    )
    tower.visual(
        Box((shaft_size, shaft_size, shaft_height)),
        origin=Origin(xyz=(0.0, 0.0, shaft_center_z)),
        material=glass,
        name="square_glass_shaft",
    )

    post_w = 0.105
    post_z = shaft_center_z
    for ix, x in enumerate((-shaft_size / 2.0 - post_w / 2.8, shaft_size / 2.0 + post_w / 2.8)):
        for iy, y in enumerate((-shaft_size / 2.0 - post_w / 2.8, shaft_size / 2.0 + post_w / 2.8)):
            tower.visual(
                Box((post_w, post_w, shaft_height + 0.18)),
                origin=Origin(xyz=(x, y, post_z)),
                material=steel,
                name=f"corner_post_{ix}_{iy}",
            )

    band_zs = [1.25, 2.35, 3.45, 4.55, 5.65, 6.75, 7.65]
    band_depth = 0.075
    for i, z in enumerate(band_zs):
        tower.visual(
            Box((shaft_size + 0.28, band_depth, 0.075)),
            origin=Origin(xyz=(0.0, -shaft_size / 2.0 - band_depth / 2.0, z)),
            material=steel,
            name=f"front_band_{i}",
        )
        tower.visual(
            Box((shaft_size + 0.28, band_depth, 0.075)),
            origin=Origin(xyz=(0.0, shaft_size / 2.0 + band_depth / 2.0, z)),
            material=steel,
            name=f"rear_band_{i}",
        )
        tower.visual(
            Box((band_depth, shaft_size + 0.28, 0.075)),
            origin=Origin(xyz=(-shaft_size / 2.0 - band_depth / 2.0, 0.0, z)),
            material=steel,
            name=f"side_band_{i}_0",
        )
        tower.visual(
            Box((band_depth, shaft_size + 0.28, 0.075)),
            origin=Origin(xyz=(shaft_size / 2.0 + band_depth / 2.0, 0.0, z)),
            material=steel,
            name=f"side_band_{i}_1",
        )

    tower.visual(
        Box((head_size, head_size, head_height)),
        origin=Origin(xyz=(0.0, 0.0, head_center_z)),
        material=glass,
        name="glass_clock_room",
    )
    tower.visual(
        Box((head_size + 0.30, head_size + 0.30, 0.15)),
        origin=Origin(xyz=(0.0, 0.0, head_center_z - head_height / 2.0 - 0.02)),
        material=steel,
        name="lower_head_belt",
    )
    tower.visual(
        Box((head_size + 0.36, head_size + 0.36, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, head_center_z + head_height / 2.0 + 0.02)),
        material=steel,
        name="upper_head_belt",
    )
    tower.visual(
        Box((1.42, 1.42, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, head_center_z + head_height / 2.0 + 0.20)),
        material=glass,
        name="glass_lantern",
    )
    tower.visual(
        Box((1.66, 1.66, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, head_center_z + head_height / 2.0 + 0.36)),
        material=steel,
        name="flat_roof_cap",
    )
    tower.visual(
        Cylinder(radius=0.025, length=1.15),
        origin=Origin(xyz=(0.0, 0.0, head_center_z + head_height / 2.0 + 0.96)),
        material=steel,
        name="antenna_mast",
    )

    # Face-local axes: local X is to the viewer's right, local Y is upward, and
    # local Z is the face normal.  The rpy values place cylinders, rims, and the
    # rotating hand frames in those same local clock-face coordinates.
    face_specs = [
        {
            "name": "face_0",
            "center": (0.0, -head_size / 2.0 - 0.020, face_center_z),
            "rpy": (math.pi / 2.0, 0.0, 0.0),
            "normal_axis": "y",
            "normal_sign": -1.0,
        },
        {
            "name": "face_1",
            "center": (0.0, head_size / 2.0 + 0.020, face_center_z),
            "rpy": (math.pi / 2.0, 0.0, math.pi),
            "normal_axis": "y",
            "normal_sign": 1.0,
        },
        {
            "name": "face_2",
            "center": (head_size / 2.0 + 0.020, 0.0, face_center_z),
            "rpy": (math.pi / 2.0, 0.0, math.pi / 2.0),
            "normal_axis": "x",
            "normal_sign": 1.0,
        },
        {
            "name": "face_3",
            "center": (-head_size / 2.0 - 0.020, 0.0, face_center_z),
            "rpy": (math.pi / 2.0, 0.0, -math.pi / 2.0),
            "normal_axis": "x",
            "normal_sign": -1.0,
        },
    ]

    for spec_index, spec in enumerate(face_specs):
        name = spec["name"]
        cx, cy, cz = spec["center"]
        rpy = spec["rpy"]
        axis = spec["normal_axis"]
        sign = spec["normal_sign"]

        def offset_along_normal(distance: float) -> tuple[float, float, float]:
            if axis == "x":
                return (cx + sign * distance, cy, cz)
            return (cx, cy + sign * distance, cz)

        tower.visual(
            Cylinder(radius=0.590, length=0.040),
            origin=Origin(xyz=offset_along_normal(0.000), rpy=rpy),
            material=frosted_glass,
            name=f"dial_glass_{spec_index}",
        )
        tower.visual(
            dial_ring_mesh,
            origin=Origin(xyz=offset_along_normal(0.004), rpy=rpy),
            material=steel,
            name=f"steel_rim_{spec_index}",
        )
        tower.visual(
            marker_mesh,
            origin=Origin(xyz=offset_along_normal(0.027), rpy=rpy),
            material=warm_led,
            name=f"hour_marks_{spec_index}",
        )

        # Two independent, coaxial continuous hands per face.  The hand geometry
        # is written in face-local coordinates: Y points to twelve o'clock, Z is
        # outward from the face.  The minute hand sits slightly proud of the hour
        # hand, like a real stacked clock movement.
        hour = model.part(f"hour_hand_{spec_index}")
        hour.visual(
            Box((0.080, 0.400, 0.012)),
            origin=Origin(xyz=(0.0, 0.260, 0.030)),
            material=dark_steel,
            name="hour_blade",
        )
        hour.visual(
            hour_hub_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
            material=dark_steel,
            name="hour_hub",
        )
        hour.visual(
            Box((0.038, 0.110, 0.010)),
            origin=Origin(xyz=(0.0, -0.100, 0.030)),
            material=dark_steel,
            name="hour_tail",
        )

        minute = model.part(f"minute_hand_{spec_index}")
        minute.visual(
            Cylinder(radius=0.020, length=0.075),
            origin=Origin(xyz=(0.0, 0.0, 0.0375)),
            material=dark_steel,
            name="minute_spindle",
        )
        minute.visual(
            Box((0.044, 0.615, 0.006)),
            origin=Origin(xyz=(0.0, 0.285, 0.066)),
            material=dark_steel,
            name="minute_blade",
        )
        minute.visual(
            Cylinder(radius=0.052, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.066)),
            material=dark_steel,
            name="minute_hub",
        )
        minute.visual(
            Box((0.026, 0.185, 0.005)),
            origin=Origin(xyz=(0.0, -0.072, 0.066)),
            material=dark_steel,
            name="minute_tail",
        )

        joint_xyz = offset_along_normal(0.020)
        model.articulation(
            f"face_{spec_index}_hour",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=hour,
            origin=Origin(xyz=joint_xyz, rpy=rpy),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.35),
            motion_properties=MotionProperties(damping=0.01, friction=0.002),
        )
        model.articulation(
            f"face_{spec_index}_minute",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=minute,
            origin=Origin(xyz=joint_xyz, rpy=rpy),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=6.4),
            motion_properties=MotionProperties(damping=0.005, friction=0.001),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    joints = object_model.articulations
    continuous_hands = [joint for joint in joints if joint.articulation_type == ArticulationType.CONTINUOUS]
    ctx.check(
        "four faces each have two continuous hands",
        len(continuous_hands) == 8,
        details=f"continuous hand joints={len(continuous_hands)}",
    )

    normal_checks = [
        ("y", "hour_hand_0", "minute_hand_0"),
        ("y", "minute_hand_1", "hour_hand_1"),
        ("x", "minute_hand_2", "hour_hand_2"),
        ("x", "hour_hand_3", "minute_hand_3"),
    ]
    for face_index in range(4):
        hour = object_model.get_part(f"hour_hand_{face_index}")
        minute = object_model.get_part(f"minute_hand_{face_index}")
        hour_joint = object_model.get_articulation(f"face_{face_index}_hour")
        minute_joint = object_model.get_articulation(f"face_{face_index}_minute")

        ctx.check(
            f"face {face_index} hour joint is continuous",
            hour_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=str(hour_joint.articulation_type),
        )
        ctx.check(
            f"face {face_index} minute joint is continuous",
            minute_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=str(minute_joint.articulation_type),
        )
        ctx.expect_origin_distance(
            hour,
            minute,
            axes="xyz",
            max_dist=0.001,
            name=f"face {face_index} hands share center axis",
        )

        axis, positive_name, negative_name = normal_checks[face_index]
        ctx.expect_gap(
            object_model.get_part(positive_name),
            object_model.get_part(negative_name),
            axis=axis,
            positive_elem="hour_blade" if "hour" in positive_name else "minute_blade",
            negative_elem="hour_blade" if "hour" in negative_name else "minute_blade",
            min_gap=0.004,
            name=f"face {face_index} stacked hands are separated",
        )

    minute_0 = object_model.get_part("minute_hand_0")
    minute_joint_0 = object_model.get_articulation("face_0_minute")
    rest_aabb = ctx.part_element_world_aabb(minute_0, elem="minute_blade")
    with ctx.pose({minute_joint_0: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(minute_0, elem="minute_blade")
    if rest_aabb is None or turned_aabb is None:
        ctx.fail("minute hand produces measurable rotating blade", "missing minute blade AABB")
    else:
        rest_width = rest_aabb[1][0] - rest_aabb[0][0]
        turned_width = turned_aabb[1][0] - turned_aabb[0][0]
        ctx.check(
            "minute hand visibly rotates in the clock plane",
            turned_width > rest_width + 0.35,
            details=f"rest_width={rest_width:.3f}, turned_width={turned_width:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
