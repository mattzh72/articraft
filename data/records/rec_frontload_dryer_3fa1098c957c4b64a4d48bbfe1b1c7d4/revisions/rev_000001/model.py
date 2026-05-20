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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def annular_shell_geometry(
        inner_radius: float,
        outer_radius: float,
        depth: float,
        *,
        outer_bevel: float = 0.0,
        inner_bevel: float = 0.0,
        segments: int = 72,
    ):
        half_depth = depth * 0.5
        outer = [
            (outer_radius - outer_bevel, -half_depth),
            (outer_radius, -half_depth * 0.30),
            (outer_radius, half_depth * 0.30),
            (outer_radius - outer_bevel, half_depth),
        ]
        inner = [
            (inner_radius + inner_bevel, -half_depth),
            (inner_radius, -half_depth * 0.30),
            (inner_radius, half_depth * 0.30),
            (inner_radius + inner_bevel, half_depth),
        ]
        return LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        )

    def drum_shell_geometry():
        return LatheGeometry.from_shell_profiles(
            [
                (0.340, -0.360),
                (0.355, -0.336),
                (0.355, 0.336),
                (0.340, 0.360),
            ],
            [
                (0.320, -0.360),
                (0.333, -0.336),
                (0.333, 0.336),
                (0.320, 0.360),
            ],
            segments=84,
            start_cap="flat",
            end_cap="flat",
        )

    model = ArticulatedObject(name="industrial_dryer")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    darker_stainless = model.material("darker_stainless", rgba=(0.58, 0.61, 0.65, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.60, 0.76, 0.82, 0.34))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((1.18, 0.89, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=darker_stainless,
        name="base_pan",
    )
    cabinet.visual(
        Box((0.030, 0.92, 1.52)),
        origin=Origin(xyz=(-0.585, 0.0, 0.815)),
        material=stainless,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((0.030, 0.92, 1.52)),
        origin=Origin(xyz=(0.585, 0.0, 0.815)),
        material=stainless,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((1.14, 0.90, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 1.635)),
        material=stainless,
        name="top_panel",
    )
    cabinet.visual(
        Box((1.14, 0.028, 1.56)),
        origin=Origin(xyz=(0.0, -0.461, 0.8075)),
        material=stainless,
        name="back_panel",
    )
    cabinet.visual(
        Box((1.14, 0.028, 0.420)),
        origin=Origin(xyz=(0.0, 0.461, 1.430)),
        material=stainless,
        name="front_top_strip",
    )
    cabinet.visual(
        Box((1.14, 0.028, 0.350)),
        origin=Origin(xyz=(0.0, 0.461, 0.205)),
        material=stainless,
        name="front_bottom_strip",
    )
    cabinet.visual(
        Box((0.160, 0.028, 0.860)),
        origin=Origin(xyz=(-0.505, 0.461, 0.790)),
        material=stainless,
        name="front_left_strip",
    )
    cabinet.visual(
        Box((0.160, 0.028, 0.860)),
        origin=Origin(xyz=(0.505, 0.461, 0.790)),
        material=stainless,
        name="front_right_strip",
    )
    cabinet.visual(
        save_mesh(
            "dryer_front_bezel",
            annular_shell_geometry(
                inner_radius=0.352,
                outer_radius=0.440,
                depth=0.032,
                outer_bevel=0.010,
                inner_bevel=0.006,
            ).rotate_x(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.0, 0.460, 0.790)),
        material=stainless,
        name="front_bezel",
    )
    cabinet.visual(
        save_mesh(
            "dryer_front_gasket",
            annular_shell_geometry(
                inner_radius=0.312,
                outer_radius=0.356,
                depth=0.008,
                outer_bevel=0.002,
                inner_bevel=0.002,
            ).rotate_x(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.0, 0.471, 0.790)),
        material=gasket_black,
        name="front_gasket",
    )
    cabinet.visual(
        Box((0.820, 0.160, 0.140)),
        origin=Origin(xyz=(0.0, 0.365, 1.555)),
        material=darker_stainless,
        name="control_console",
    )
    cabinet.visual(
        Box((0.560, 0.010, 0.085)),
        origin=Origin(xyz=(0.0, 0.448, 1.555)),
        material=graphite,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.028, 0.018, 0.620)),
        origin=Origin(xyz=(-0.468, 0.476, 0.790)),
        material=darker_stainless,
        name="hinge_spine",
    )
    cabinet.visual(
        Box((0.048, 0.026, 0.150)),
        origin=Origin(xyz=(-0.447, 0.492, 0.980)),
        material=darker_stainless,
        name="upper_hinge_bracket",
    )
    cabinet.visual(
        Box((0.048, 0.026, 0.150)),
        origin=Origin(xyz=(-0.447, 0.492, 0.600)),
        material=darker_stainless,
        name="lower_hinge_bracket",
    )
    cabinet.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(xyz=(-0.405, 0.505, 0.990)),
        material=graphite,
        name="upper_hinge_barrel",
    )
    cabinet.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(xyz=(-0.405, 0.505, 0.600)),
        material=graphite,
        name="lower_hinge_barrel",
    )
    cabinet.visual(
        Box((0.040, 0.018, 0.130)),
        origin=Origin(xyz=(0.404, 0.450, 0.790)),
        material=graphite,
        name="latch_strike",
    )
    cabinet.visual(
        Box((0.220, 0.060, 0.317)),
        origin=Origin(xyz=(0.0, -0.180, 0.2135)),
        material=graphite,
        name="rear_support_pedestal",
    )
    cabinet.visual(
        Box((0.220, 0.060, 0.317)),
        origin=Origin(xyz=(0.0, 0.180, 0.2135)),
        material=graphite,
        name="front_support_pedestal",
    )
    cabinet.visual(
        Box((0.220, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.180, 0.392)),
        material=darker_stainless,
        name="rear_support_rail",
    )
    cabinet.visual(
        Box((0.220, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.180, 0.392)),
        material=darker_stainless,
        name="front_support_rail",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        ((-0.48, -0.34), (-0.48, 0.34), (0.48, -0.34), (0.48, 0.34))
    ):
        cabinet.visual(
            Cylinder(radius=0.040, length=0.040),
            origin=Origin(xyz=(foot_x, foot_y, -0.020)),
            material=graphite,
            name=f"foot_{foot_index}",
        )
    cabinet.inertial = Inertial.from_geometry(
        Box((1.20, 0.95, 1.65)),
        mass=285.0,
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
    )

    drum = model.part("drum")
    drum.visual(
        save_mesh("dryer_drum_shell", drum_shell_geometry().rotate_x(math.pi / 2.0)),
        material=darker_stainless,
        name="drum_shell",
    )
    drum.visual(
        save_mesh(
            "dryer_front_support_band",
            annular_shell_geometry(
                inner_radius=0.340,
                outer_radius=0.368,
                depth=0.050,
                outer_bevel=0.004,
                inner_bevel=0.002,
            ).rotate_x(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.0, 0.180, 0.0)),
        material=graphite,
        name="front_support_band",
    )
    drum.visual(
        save_mesh(
            "dryer_rear_support_band",
            annular_shell_geometry(
                inner_radius=0.340,
                outer_radius=0.368,
                depth=0.050,
                outer_bevel=0.004,
                inner_bevel=0.002,
            ).rotate_x(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.0, -0.180, 0.0)),
        material=graphite,
        name="rear_support_band",
    )
    drum.visual(
        save_mesh(
            "dryer_drum_front_lip",
            annular_shell_geometry(
                inner_radius=0.240,
                outer_radius=0.345,
                depth=0.038,
                outer_bevel=0.008,
                inner_bevel=0.004,
            ).rotate_x(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.0, 0.338, 0.0)),
        material=darker_stainless,
        name="front_lip",
    )
    drum.visual(
        Cylinder(radius=0.336, length=0.024),
        origin=Origin(xyz=(0.0, -0.348, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_stainless,
        name="rear_bulkhead",
    )
    drum.visual(
        Cylinder(radius=0.080, length=0.120),
        origin=Origin(xyz=(0.0, -0.275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.032, length=0.080),
        origin=Origin(xyz=(0.0, -0.375, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="axle_stub",
    )
    for baffle_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.060, 0.540, 0.090)),
            origin=Origin(
                xyz=(0.294 * math.sin(angle), 0.0, 0.294 * math.cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=stainless,
            name=f"baffle_{baffle_index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.37, length=0.72),
        mass=92.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        save_mesh(
            "dryer_door_outer_ring",
            annular_shell_geometry(
                inner_radius=0.294,
                outer_radius=0.396,
                depth=0.060,
                outer_bevel=0.012,
                inner_bevel=0.006,
            ).rotate_x(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.405, 0.0, 0.0)),
        material=stainless,
        name="outer_ring",
    )
    door.visual(
        save_mesh(
            "dryer_door_inner_ring",
            annular_shell_geometry(
                inner_radius=0.280,
                outer_radius=0.310,
                depth=0.016,
                outer_bevel=0.003,
                inner_bevel=0.002,
            ).rotate_x(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.405, -0.015, 0.0)),
        material=gasket_black,
        name="inner_ring",
    )
    door.visual(
        Cylinder(radius=0.304, length=0.016),
        origin=Origin(xyz=(0.405, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="glass_window",
    )
    door.visual(
        Box((0.020, 0.024, 0.430)),
        origin=Origin(xyz=(0.052, -0.020, 0.0)),
        material=darker_stainless,
        name="hinge_rail",
    )
    door.visual(
        Box((0.050, 0.024, 0.090)),
        origin=Origin(xyz=(0.045, -0.020, 0.175)),
        material=darker_stainless,
        name="upper_hinge_tab",
    )
    door.visual(
        Box((0.050, 0.024, 0.090)),
        origin=Origin(xyz=(0.045, -0.020, -0.175)),
        material=darker_stainless,
        name="lower_hinge_tab",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.078),
        origin=Origin(xyz=(0.702, 0.050, 0.108), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="handle_post_upper",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.078),
        origin=Origin(xyz=(0.702, 0.050, -0.108), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="handle_post_lower",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.280),
        origin=Origin(xyz=(0.702, 0.095, 0.0)),
        material=graphite,
        name="bar_pull",
    )
    door.visual(
        Box((0.034, 0.028, 0.092)),
        origin=Origin(xyz=(0.782, -0.014, 0.0)),
        material=graphite,
        name="latch_tongue",
    )
    door.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.405, -0.025, 0.264), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="seal_pad_top",
    )
    door.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.405, -0.025, -0.264), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="seal_pad_bottom",
    )
    door.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.662, -0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="seal_pad_right",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.82, 0.12, 0.82)),
        mass=24.0,
        origin=Origin(xyz=(0.405, 0.0, 0.0)),
    )

    model.articulation(
        "drum_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=4.0),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.405, 0.505, 0.790)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=0.0,
            upper=1.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_spin = object_model.get_articulation("drum_spin")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    def aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    def axis_matches(actual, expected):
        return all(abs(a - b) < 1e-6 for a, b in zip(actual, expected))

    ctx.check(
        "drum articulation uses front-to-back axis",
        axis_matches(drum_spin.axis, (0.0, 1.0, 0.0)),
        f"expected (0, 1, 0), got {drum_spin.axis}",
    )
    ctx.check(
        "door articulation uses vertical hinge axis",
        axis_matches(door_hinge.axis, (0.0, 0.0, 1.0)),
        f"expected (0, 0, 1), got {door_hinge.axis}",
    )
    ctx.check(
        "door opening range is realistic",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and 1.4 <= door_hinge.motion_limits.upper <= 1.9,
        f"unexpected door limits {door_hinge.motion_limits}",
    )

    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="seal_pad_right",
        negative_elem="front_gasket",
        name="door closes onto cabinet gasket",
    )
    ctx.expect_gap(
        drum,
        cabinet,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="front_support_band",
        negative_elem="front_support_rail",
        name="front drum band sits on front rail",
    )
    ctx.expect_gap(
        drum,
        cabinet,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="rear_support_band",
        negative_elem="rear_support_rail",
        name="rear drum band sits on rear rail",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="z",
        min_overlap=0.60,
        elem_a="glass_window",
        elem_b="front_gasket",
        name="door window aligns with front opening height",
    )

    ring_rest = ctx.part_element_world_aabb(door, elem="outer_ring")
    assert ring_rest is not None
    with ctx.pose({door_hinge: 1.30}):
        ring_open = ctx.part_element_world_aabb(door, elem="outer_ring")
        assert ring_open is not None
        ring_rest_center = aabb_center(ring_rest)
        ring_open_center = aabb_center(ring_open)
        ctx.check(
            "door swings outward and left from hinge line",
            ring_open_center[1] > ring_rest_center[1] + 0.18
            and ring_open_center[0] < ring_rest_center[0] - 0.16,
            f"rest center {ring_rest_center}, open center {ring_open_center}",
        )
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.55,
            positive_elem="latch_tongue",
            negative_elem="front_gasket",
            name="open door clears cabinet face on latch side",
        )

    baffle_rest = ctx.part_element_world_aabb(drum, elem="baffle_0")
    assert baffle_rest is not None
    with ctx.pose({drum_spin: math.pi / 2.0}):
        baffle_spun = ctx.part_element_world_aabb(drum, elem="baffle_0")
        assert baffle_spun is not None
        baffle_rest_center = aabb_center(baffle_rest)
        baffle_spun_center = aabb_center(baffle_spun)
        ctx.check(
            "drum internals rotate with drum",
            baffle_spun_center[0] > baffle_rest_center[0] + 0.18
            and baffle_spun_center[2] < baffle_rest_center[2] - 0.12,
            f"rest center {baffle_rest_center}, spun center {baffle_spun_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
