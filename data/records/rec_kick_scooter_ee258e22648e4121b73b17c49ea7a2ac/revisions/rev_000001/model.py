from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="off_road_kick_scooter")

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(x, -y, z) for x, y, z in points]

    def _add_pneumatic_wheel(
        part,
        *,
        mesh_prefix: str,
        tire_radius: float = 0.135,
        tire_width: float = 0.055,
        rim_radius: float = 0.085,
        hub_radius: float = 0.026,
        hub_width: float = 0.050,
        axle_width: float = 0.092,
        rubber=None,
        metal=None,
        dark_metal=None,
    ) -> None:
        half_width = tire_width * 0.5
        tire_profile = [
            (0.094, -half_width * 0.96),
            (0.110, -half_width * 0.99),
            (0.126, -half_width * 0.78),
            (tire_radius, -half_width * 0.24),
            (tire_radius, half_width * 0.24),
            (0.126, half_width * 0.78),
            (0.110, half_width * 0.99),
            (0.094, half_width * 0.96),
            (0.086, half_width * 0.42),
            (0.079, 0.0),
            (0.086, -half_width * 0.42),
            (0.094, -half_width * 0.96),
        ]
        tire_mesh = _mesh(
            f"{mesh_prefix}_tire",
            LatheGeometry(tire_profile, segments=60).rotate_x(pi / 2.0),
        )
        part.visual(tire_mesh, material=rubber, name="tire")
        part.visual(
            Cylinder(radius=rim_radius, length=tire_width * 0.62),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="rim",
        )
        part.visual(
            Cylinder(radius=hub_radius, length=hub_width),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hub",
        )
        part.visual(
            Cylinder(radius=0.010, length=axle_width),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="axle_spacer",
        )
        spoke_length = 0.060
        spoke_center = 0.053
        for spoke_index in range(6):
            angle = 2.0 * pi * spoke_index / 6.0
            part.visual(
                Box((spoke_length, 0.008, 0.018)),
                origin=Origin(xyz=(spoke_center * cos(angle), 0.0, spoke_center * sin(angle)), rpy=(0.0, angle, 0.0)),
                material=metal,
                name=f"spoke_{spoke_index}",
            )

    model.material("frame_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("deck_charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("metal_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    model.material("dark_metal", rgba=(0.30, 0.31, 0.33, 1.0))
    model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))

    frame_black = "frame_black"
    deck_charcoal = "deck_charcoal"
    metal_gray = "metal_gray"
    dark_metal = "dark_metal"
    rubber_black = "rubber_black"
    grip_black = "grip_black"

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.92, 0.18, 0.24)),
        mass=8.8,
        origin=Origin(xyz=(-0.02, 0.0, 0.07)),
    )

    deck_shell = _mesh(
        "deck_shell",
        ExtrudeGeometry(
            rounded_rect_profile(0.60, 0.15, 0.028, corner_segments=8),
            0.026,
            center=True,
        ),
    )
    frame.visual(deck_shell, material=deck_charcoal, name="deck_shell")
    frame.visual(
        Box((0.50, 0.10, 0.004)),
        origin=Origin(xyz=(0.00, 0.00, 0.015)),
        material=grip_black,
        name="grip_pad",
    )
    frame.visual(
        Box((0.040, 0.100, 0.164)),
        origin=Origin(xyz=(0.280, 0.000, 0.095)),
        material=frame_black,
        name="head_block",
    )
    frame.visual(
        Box((0.120, 0.014, 0.120)),
        origin=Origin(xyz=(0.235, 0.044, 0.067)),
        material=frame_black,
        name="left_head_gusset",
    )
    frame.visual(
        Box((0.120, 0.014, 0.120)),
        origin=Origin(xyz=(0.235, -0.044, 0.067)),
        material=frame_black,
        name="right_head_gusset",
    )

    left_rail = _mesh(
        "left_rear_rail",
        tube_from_spline_points(
            [(-0.215, 0.057, -0.004), (-0.330, 0.064, 0.010), (-0.460, 0.070, 0.026)],
            radius=0.014,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    right_rail = _mesh(
        "right_rear_rail",
        tube_from_spline_points(
            _mirror_y([(-0.215, 0.057, -0.004), (-0.330, 0.064, 0.010), (-0.460, 0.070, 0.026)]),
            radius=0.014,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    frame.visual(left_rail, material=frame_black, name="left_rear_rail")
    frame.visual(right_rail, material=frame_black, name="right_rear_rail")
    frame.visual(
        Box((0.090, 0.085, 0.020)),
        origin=Origin(xyz=(-0.245, 0.000, 0.010)),
        material=frame_black,
        name="rear_bridge",
    )
    frame.visual(
        Box((0.016, 0.014, 0.074)),
        origin=Origin(xyz=(-0.455, 0.053, 0.028)),
        material=dark_metal,
        name="rear_dropout_left",
    )
    frame.visual(
        Box((0.016, 0.014, 0.074)),
        origin=Origin(xyz=(-0.455, -0.053, 0.028)),
        material=dark_metal,
        name="rear_dropout_right",
    )
    frame.visual(
        Box((0.050, 0.038, 0.008)),
        origin=Origin(xyz=(0.020, -0.060, -0.019)),
        material=frame_black,
        name="stand_mount_base",
    )
    frame.visual(
        Box((0.010, 0.006, 0.020)),
        origin=Origin(xyz=(0.003, -0.077, -0.008)),
        material=dark_metal,
        name="stand_web_rear",
    )
    frame.visual(
        Box((0.010, 0.006, 0.020)),
        origin=Origin(xyz=(0.037, -0.077, -0.008)),
        material=dark_metal,
        name="stand_web_front",
    )
    frame.visual(
        Box((0.010, 0.022, 0.024)),
        origin=Origin(xyz=(0.003, -0.090, -0.014)),
        material=dark_metal,
        name="stand_tab_rear",
    )
    frame.visual(
        Box((0.010, 0.022, 0.024)),
        origin=Origin(xyz=(0.037, -0.090, -0.014)),
        material=dark_metal,
        name="stand_tab_front",
    )

    front_fork = model.part("front_fork")
    front_fork.inertial = Inertial.from_geometry(
        Box((0.160, 0.560, 0.700)),
        mass=3.9,
        origin=Origin(xyz=(0.045, 0.0, 0.200)),
    )
    front_fork.visual(
        Cylinder(radius=0.022, length=0.630),
        origin=Origin(xyz=(0.000, 0.000, 0.315)),
        material=frame_black,
        name="steering_tube",
    )
    front_fork.visual(
        Box((0.080, 0.050, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 0.565)),
        material=dark_metal,
        name="stem_clamp",
    )
    front_fork.visual(
        Cylinder(radius=0.017, length=0.520),
        origin=Origin(xyz=(0.000, 0.000, 0.605), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handlebar_bar",
    )
    front_fork.visual(
        Cylinder(radius=0.019, length=0.100),
        origin=Origin(xyz=(0.000, 0.255, 0.605), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="left_grip",
    )
    front_fork.visual(
        Cylinder(radius=0.019, length=0.100),
        origin=Origin(xyz=(0.000, -0.255, 0.605), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="right_grip",
    )
    front_fork.visual(
        Box((0.130, 0.090, 0.040)),
        origin=Origin(xyz=(0.080, 0.000, 0.005)),
        material=frame_black,
        name="lower_crown",
    )
    front_fork.visual(
        Box((0.040, 0.110, 0.020)),
        origin=Origin(xyz=(0.120, 0.000, -0.005)),
        material=frame_black,
        name="fork_bridge",
    )
    front_fork.visual(
        Cylinder(radius=0.015, length=0.126),
        origin=Origin(xyz=(0.120, 0.046, -0.072)),
        material=metal_gray,
        name="left_leg",
    )
    front_fork.visual(
        Cylinder(radius=0.015, length=0.126),
        origin=Origin(xyz=(0.120, -0.046, -0.072)),
        material=metal_gray,
        name="right_leg",
    )
    front_fork.visual(
        Box((0.030, 0.018, 0.032)),
        origin=Origin(xyz=(0.120, 0.055, -0.151)),
        material=dark_metal,
        name="left_clamp",
    )
    front_fork.visual(
        Box((0.030, 0.018, 0.032)),
        origin=Origin(xyz=(0.120, -0.055, -0.151)),
        material=dark_metal,
        name="right_clamp",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.055),
        mass=1.9,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_pneumatic_wheel(
        front_wheel,
        mesh_prefix="front_wheel",
        rubber=rubber_black,
        metal=metal_gray,
        dark_metal=dark_metal,
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.055),
        mass=1.9,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_pneumatic_wheel(
        rear_wheel,
        mesh_prefix="rear_wheel",
        rubber=rubber_black,
        metal=metal_gray,
        dark_metal=dark_metal,
    )

    side_stand = model.part("side_stand")
    side_stand.inertial = Inertial.from_geometry(
        Box((0.035, 0.210, 0.050)),
        mass=0.35,
        origin=Origin(xyz=(0.000, -0.105, -0.022)),
    )
    side_stand.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="stand_barrel",
    )
    side_stand.visual(
        _mesh(
            "side_stand_leg",
            tube_from_spline_points(
                [(0.0, 0.0, 0.0), (0.0, -0.035, -0.010), (0.0, -0.185, -0.040)],
                radius=0.009,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=dark_metal,
        name="stand_leg",
    )
    side_stand.visual(
        Box((0.045, 0.016, 0.008)),
        origin=Origin(xyz=(0.000, -0.190, -0.040)),
        material=dark_metal,
        name="stand_foot",
    )

    model.articulation(
        "frame_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_fork,
        origin=Origin(xyz=(0.320, 0.000, 0.175)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.8, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "front_fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.120, 0.000, -0.152)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
    )
    model.articulation(
        "frame_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel,
        origin=Origin(xyz=(-0.455, 0.000, 0.023)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
    )
    model.articulation(
        "frame_to_side_stand",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=side_stand,
        origin=Origin(xyz=(0.020, -0.090, -0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.10),
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

    frame = object_model.get_part("frame")
    front_fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    side_stand = object_model.get_part("side_stand")

    steer = object_model.get_articulation("frame_to_front_fork")
    front_spin = object_model.get_articulation("front_fork_to_front_wheel")
    rear_spin = object_model.get_articulation("frame_to_rear_wheel")
    stand_hinge = object_model.get_articulation("frame_to_side_stand")

    ctx.check(
        "steering axis is vertical",
        tuple(steer.axis) == (0.0, 0.0, 1.0),
        details=f"axis={steer.axis}",
    )
    ctx.check(
        "wheel spin axes are lateral",
        tuple(front_spin.axis) == (0.0, 1.0, 0.0) and tuple(rear_spin.axis) == (0.0, 1.0, 0.0),
        details=f"front={front_spin.axis}, rear={rear_spin.axis}",
    )
    ctx.check(
        "side stand hinge axis is fore aft",
        tuple(stand_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={stand_hinge.axis}",
    )

    with ctx.pose({steer: 0.0, stand_hinge: 0.0}):
        ctx.expect_gap(
            front_fork,
            front_wheel,
            axis="y",
            positive_elem="left_clamp",
            negative_elem="axle_spacer",
            min_gap=0.0,
            max_gap=0.0,
            name="front wheel spacer seats against left fork clamp",
        )
        ctx.expect_gap(
            front_wheel,
            front_fork,
            axis="y",
            positive_elem="axle_spacer",
            negative_elem="right_clamp",
            min_gap=0.0,
            max_gap=0.0,
            name="front wheel spacer seats against right fork clamp",
        )
        ctx.expect_contact(
            rear_wheel,
            frame,
            elem_a="axle_spacer",
            elem_b="rear_dropout_left",
            name="rear wheel spacer meets left dropout",
        )
        ctx.expect_contact(
            rear_wheel,
            frame,
            elem_a="axle_spacer",
            elem_b="rear_dropout_right",
            name="rear wheel spacer meets right dropout",
        )
        ctx.expect_gap(
            frame,
            side_stand,
            axis="x",
            positive_elem="stand_tab_front",
            negative_elem="stand_barrel",
            min_gap=0.0,
            max_gap=0.0,
            name="side stand barrel sits against front tab",
        )
        ctx.expect_gap(
            side_stand,
            frame,
            axis="x",
            positive_elem="stand_barrel",
            negative_elem="stand_tab_rear",
            min_gap=0.0,
            max_gap=0.0,
            name="side stand barrel sits against rear tab",
        )
        ctx.expect_origin_gap(
            front_wheel,
            rear_wheel,
            axis="x",
            min_gap=0.78,
            max_gap=0.90,
            name="wheelbase matches a long off road scooter",
        )

    front_rest = ctx.part_world_position(front_wheel)
    with ctx.pose({steer: 0.45}):
        front_turned = ctx.part_world_position(front_wheel)
    ctx.check(
        "front fork steers the wheel sideways",
        front_rest is not None
        and front_turned is not None
        and abs(front_turned[1] - front_rest[1]) > 0.03
        and front_turned[0] < front_rest[0] - 0.004,
        details=f"rest={front_rest}, turned={front_turned}",
    )

    spin_rest_front = ctx.part_world_position(front_wheel)
    spin_rest_rear = ctx.part_world_position(rear_wheel)
    with ctx.pose({front_spin: 1.2, rear_spin: -0.9}):
        spin_pose_front = ctx.part_world_position(front_wheel)
        spin_pose_rear = ctx.part_world_position(rear_wheel)
    ctx.check(
        "wheel spin stays on the axle centers",
        spin_rest_front is not None
        and spin_rest_rear is not None
        and spin_pose_front is not None
        and spin_pose_rear is not None
        and max(abs(a - b) for a, b in zip(spin_rest_front, spin_pose_front)) < 1e-6
        and max(abs(a - b) for a, b in zip(spin_rest_rear, spin_pose_rear)) < 1e-6,
        details=(
            f"front_rest={spin_rest_front}, front_spin={spin_pose_front}, "
            f"rear_rest={spin_rest_rear}, rear_spin={spin_pose_rear}"
        ),
    )

    folded_aabb = ctx.part_world_aabb(side_stand)
    with ctx.pose({stand_hinge: 1.00}):
        deployed_aabb = ctx.part_world_aabb(side_stand)
    ctx.check(
        "side stand swings downward when deployed",
        folded_aabb is not None
        and deployed_aabb is not None
        and deployed_aabb[0][2] < folded_aabb[0][2] - 0.10,
        details=f"folded={folded_aabb}, deployed={deployed_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
