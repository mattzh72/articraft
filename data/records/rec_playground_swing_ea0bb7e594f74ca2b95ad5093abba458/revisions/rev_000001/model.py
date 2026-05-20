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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _belt_section(
    x_pos: float,
    *,
    z_center: float,
    depth: float,
    thickness: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(depth, thickness, corner_radius, corner_segments=6)
    return [(x_pos, y, z_center + z) for y, z in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_belt_swing")

    frame_paint = model.material("frame_paint", rgba=(0.20, 0.35, 0.23, 1.0))
    galvanized = model.material("galvanized", rgba=(0.72, 0.74, 0.76, 1.0))
    chain_steel = model.material("chain_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.18, 0.19, 0.20, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((2.70, 1.35, 2.25)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
    )

    frame.visual(
        Cylinder(radius=0.070, length=2.55),
        origin=Origin(xyz=(0.0, 0.0, 2.20), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="top_beam",
    )

    leg_length = 2.25
    leg_angle = math.radians(16.0)
    leg_z = 1.07
    front_leg_y = 0.33
    rear_leg_y = -0.33
    leg_xs = (-1.06, 1.06)
    for side_name, leg_x in (("left", leg_xs[0]), ("right", leg_xs[1])):
        frame.visual(
            Cylinder(radius=0.045, length=leg_length),
            origin=Origin(xyz=(leg_x, front_leg_y, leg_z), rpy=(leg_angle, 0.0, 0.0)),
            material=galvanized,
            name=f"{side_name}_front_leg",
        )
        frame.visual(
            Cylinder(radius=0.045, length=leg_length),
            origin=Origin(xyz=(leg_x, rear_leg_y, leg_z), rpy=(-leg_angle, 0.0, 0.0)),
            material=galvanized,
            name=f"{side_name}_rear_leg",
        )
        frame.visual(
            Cylinder(radius=0.026, length=0.54),
            origin=Origin(xyz=(leg_x, 0.0, 1.22), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=frame_paint,
            name=f"{side_name}_spreader",
        )

    frame.visual(
        Box((0.09, 0.12, 0.05)),
        origin=Origin(xyz=(-0.38, 0.0, 2.105)),
        material=dark_hardware,
        name="left_pivot_bracket",
    )
    frame.visual(
        Box((0.09, 0.12, 0.05)),
        origin=Origin(xyz=(0.38, 0.0, 2.105)),
        material=dark_hardware,
        name="right_pivot_bracket",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(-0.38, 0.0, 2.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="left_pivot_pin",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(0.38, 0.0, 2.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="right_pivot_pin",
    )

    swing = model.part("swing_assembly")
    swing.inertial = Inertial.from_geometry(
        Box((0.95, 0.32, 1.75)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.88)),
    )

    hanger_drop = 0.102
    yoke_z = -0.104
    chain_radius = 0.0055
    seat_anchor_z = -1.455
    front_chain_y = 0.065
    rear_chain_y = -0.065
    hanger_x = 0.38
    seat_anchor_x = 0.22

    swing.visual(
        Box((0.024, 0.016, hanger_drop)),
        origin=Origin(xyz=(-hanger_x, 0.0, -hanger_drop / 2.0)),
        material=galvanized,
        name="left_hanger_link",
    )
    swing.visual(
        Box((0.024, 0.120, 0.012)),
        origin=Origin(xyz=(-hanger_x, 0.0, yoke_z)),
        material=galvanized,
        name="left_hanger_yoke",
    )
    swing.visual(
        Box((0.024, 0.016, hanger_drop)),
        origin=Origin(xyz=(hanger_x, 0.0, -hanger_drop / 2.0)),
        material=galvanized,
        name="right_hanger_link",
    )
    swing.visual(
        Box((0.024, 0.120, 0.012)),
        origin=Origin(xyz=(hanger_x, 0.0, yoke_z)),
        material=galvanized,
        name="right_hanger_yoke",
    )

    chain_runs = {
        "left_front_chain": [
            (-hanger_x, front_chain_y, yoke_z),
            (-0.31, front_chain_y + 0.010, -0.80),
            (-seat_anchor_x, 0.072, seat_anchor_z),
        ],
        "left_rear_chain": [
            (-hanger_x, rear_chain_y, yoke_z),
            (-0.31, rear_chain_y - 0.010, -0.80),
            (-seat_anchor_x, -0.072, seat_anchor_z),
        ],
        "right_front_chain": [
            (hanger_x, front_chain_y, yoke_z),
            (0.31, front_chain_y + 0.010, -0.80),
            (seat_anchor_x, 0.072, seat_anchor_z),
        ],
        "right_rear_chain": [
            (hanger_x, rear_chain_y, yoke_z),
            (0.31, rear_chain_y - 0.010, -0.80),
            (seat_anchor_x, -0.072, seat_anchor_z),
        ],
    }
    for name, points in chain_runs.items():
        swing.visual(
            _save_mesh(
                name,
                tube_from_spline_points(
                    points,
                    radius=chain_radius,
                    samples_per_segment=12,
                    radial_segments=14,
                    cap_ends=True,
                ),
            ),
            material=chain_steel,
            name=name,
        )

    swing.visual(
        Box((0.034, 0.180, 0.080)),
        origin=Origin(xyz=(-seat_anchor_x, 0.0, -1.475)),
        material=dark_hardware,
        name="left_seat_bracket",
    )
    swing.visual(
        Box((0.034, 0.180, 0.080)),
        origin=Origin(xyz=(seat_anchor_x, 0.0, -1.475)),
        material=dark_hardware,
        name="right_seat_bracket",
    )

    belt_geom = section_loft(
        [
            _belt_section(-0.22, z_center=-1.485, depth=0.155, thickness=0.018, corner_radius=0.006),
            _belt_section(-0.11, z_center=-1.515, depth=0.163, thickness=0.018, corner_radius=0.006),
            _belt_section(0.00, z_center=-1.545, depth=0.170, thickness=0.018, corner_radius=0.006),
            _belt_section(0.11, z_center=-1.515, depth=0.163, thickness=0.018, corner_radius=0.006),
            _belt_section(0.22, z_center=-1.485, depth=0.155, thickness=0.018, corner_radius=0.006),
        ]
    )
    swing.visual(
        _save_mesh("belt_seat_mesh", belt_geom),
        material=rubber_black,
        name="belt_seat",
    )

    model.articulation(
        "frame_to_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing,
        origin=Origin(xyz=(0.0, 0.0, 2.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=math.radians(-40.0),
            upper=math.radians(40.0),
        ),
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
    swing = object_model.get_part("swing_assembly")
    swing_joint = object_model.get_articulation("frame_to_swing")

    top_beam = frame.get_visual("top_beam")
    left_pivot = frame.get_visual("left_pivot_bracket")
    right_pivot = frame.get_visual("right_pivot_bracket")
    belt_seat = swing.get_visual("belt_seat")
    left_hanger = swing.get_visual("left_hanger_link")
    right_hanger = swing.get_visual("right_hanger_link")

    ctx.expect_gap(
        frame,
        swing,
        axis="z",
        positive_elem=top_beam,
        negative_elem=belt_seat,
        min_gap=1.20,
        max_gap=1.55,
        name="belt seat hangs well below the top beam",
    )
    ctx.expect_contact(
        frame,
        swing,
        elem_a=left_pivot,
        elem_b=left_hanger,
        contact_tol=0.002,
        name="left hanger sits against its beam bracket",
    )
    ctx.expect_contact(
        frame,
        swing,
        elem_a=right_pivot,
        elem_b=right_hanger,
        contact_tol=0.002,
        name="right hanger sits against its beam bracket",
    )

    rest_aabb = ctx.part_element_world_aabb(swing, elem="belt_seat")
    with ctx.pose({swing_joint: math.radians(28.0)}):
        forward_aabb = ctx.part_element_world_aabb(swing, elem="belt_seat")

    def _center_yz(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return (
            0.5 * (min_corner[1] + max_corner[1]),
            0.5 * (min_corner[2] + max_corner[2]),
        )

    rest_center = _center_yz(rest_aabb)
    forward_center = _center_yz(forward_aabb)
    ctx.check(
        "positive swing pose moves the seat forward and upward",
        rest_center is not None
        and forward_center is not None
        and forward_center[0] > rest_center[0] + 0.45
        and forward_center[1] > rest_center[1] + 0.08,
        details=f"rest_center={rest_center}, forward_center={forward_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
