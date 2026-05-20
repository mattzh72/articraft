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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gas_tumble_dryer")

    body_w = 0.78
    body_d = 0.78
    body_h = 0.90
    skin_t = 0.012
    side_t = 0.016
    base_t = 0.040
    front_x = body_d * 0.5
    rear_x = -body_d * 0.5

    door_outer_r = 0.26
    door_glass_r = 0.18
    door_frame_t = 0.034
    door_depth = 0.070
    door_center_z = 0.47
    door_hinge_x = front_x + 0.029
    door_hinge_y = -door_outer_r

    drum_len = 0.56
    drum_outer_r = 0.292
    drum_inner_r = 0.276
    drum_center_x = 0.04

    gas_panel_w = 0.18
    gas_panel_h = 0.13
    gas_panel_t = 0.014
    gas_panel_y = 0.19
    gas_panel_hinge_z = 0.07

    stainless = model.material("stainless", rgba=(0.76, 0.79, 0.82, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.24, 0.29, 0.34, 0.42))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_d, body_w, body_h)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    front_panel_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(body_h, body_w, radius=0.030, corner_segments=8),
        [
            _offset_profile(
                _circle_profile(0.220, segments=56),
                dx=door_center_z - (body_h * 0.5),
                dy=0.0,
            )
        ],
        height=skin_t,
        center=True,
    )
    rear_panel_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(body_h, body_w, radius=0.026, corner_segments=8),
        [
            _offset_profile(
                rounded_rect_profile(0.104, 0.148, radius=0.012, corner_segments=6),
                dx=(gas_panel_hinge_z + gas_panel_h * 0.5) - (body_h * 0.5),
                dy=gas_panel_y,
            )
        ],
        height=skin_t,
        center=True,
    )

    body.visual(
        _save_mesh("dryer_front_panel", front_panel_geom),
        origin=Origin(
            xyz=(front_x - (skin_t * 0.5), 0.0, body_h * 0.5),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="front_panel",
    )
    body.visual(
        _save_mesh("dryer_rear_panel", rear_panel_geom),
        origin=Origin(
            xyz=(rear_x + (skin_t * 0.5), 0.0, body_h * 0.5),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="rear_panel",
    )
    body.visual(
        Box((body_d - (2.0 * skin_t), side_t, body_h - base_t)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_w * 0.5) + (side_t * 0.5),
                base_t + ((body_h - base_t) * 0.5),
            )
        ),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((body_d - (2.0 * skin_t), side_t, body_h - base_t)),
        origin=Origin(
            xyz=(
                0.0,
                (body_w * 0.5) - (side_t * 0.5),
                base_t + ((body_h - base_t) * 0.5),
            )
        ),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((body_d - (2.0 * skin_t), body_w, skin_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - (skin_t * 0.5))),
        material=stainless,
        name="top_panel",
    )
    body.visual(
        Box((body_d - (2.0 * skin_t), body_w, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t * 0.5)),
        material=dark_trim,
        name="base_pan",
    )
    body.visual(
        Box((0.060, body_w - 0.140, 0.080)),
        origin=Origin(xyz=(front_x - 0.030, 0.0, body_h - 0.070)),
        material=dark_trim,
        name="control_fascia",
    )
    body.visual(
        _save_mesh(
            "dryer_door_boot",
            TorusGeometry(radius=0.214, tube=0.010, radial_segments=18, tubular_segments=64),
        ),
        origin=Origin(
            xyz=(front_x - 0.018, 0.0, door_center_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="door_boot",
    )
    body.visual(
        Cylinder(radius=0.042, length=0.078),
        origin=Origin(
            xyz=(rear_x + 0.051, 0.0, door_center_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="rear_bearing_support",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(
            xyz=(rear_x - 0.030, 0.0, 0.635),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brushed_steel,
        name="exhaust_collar",
    )
    for index, (foot_x, foot_y) in enumerate(
        [
            (0.28, 0.31),
            (0.28, -0.31),
            (-0.28, 0.31),
            (-0.28, -0.31),
        ]
    ):
        body.visual(
            Box((0.050, 0.050, 0.045)),
            origin=Origin(xyz=(foot_x, foot_y, 0.0125)),
            material=rubber,
            name=f"foot_{index}",
        )

    drum = model.part("drum")
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_outer_r, length=drum_len),
        mass=14.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    drum_shell = LatheGeometry.from_shell_profiles(
        [
            (0.272, -0.280),
            (0.286, -0.272),
            (0.292, -0.236),
            (0.292, 0.236),
            (0.286, 0.272),
            (0.272, 0.280),
        ],
        [
            (0.246, -0.272),
            (0.276, -0.246),
            (0.276, 0.246),
            (0.246, 0.272),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    drum.visual(
        _save_mesh("dryer_drum_shell", drum_shell),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        _save_mesh(
            "dryer_drum_front_rim",
            TorusGeometry(radius=0.276, tube=0.010, radial_segments=18, tubular_segments=64),
        ),
        origin=Origin(
            xyz=(drum_len * 0.5 - 0.002, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brushed_steel,
        name="front_rim",
    )
    drum.visual(
        Cylinder(radius=0.272, length=0.024),
        origin=Origin(
            xyz=(-(drum_len * 0.5) + 0.012, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brushed_steel,
        name="rear_bulkhead",
    )
    drum.visual(
        Cylinder(radius=0.032, length=0.085),
        origin=Origin(
            xyz=(-(drum_len * 0.5) - 0.0175, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="axle_stub",
    )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((door_depth, door_outer_r * 2.0, door_outer_r * 2.0)),
        mass=5.0,
        origin=Origin(xyz=(0.0, door_outer_r, 0.0)),
    )
    door_frame_geom = ExtrudeWithHolesGeometry(
        _circle_profile(door_outer_r, segments=64),
        [_circle_profile(door_glass_r + 0.010, segments=64)],
        height=door_frame_t,
        center=True,
    )
    door.visual(
        _save_mesh("dryer_door_frame", door_frame_geom),
        origin=Origin(
            xyz=(-0.012, door_outer_r, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="door_frame",
    )
    door.visual(
        Cylinder(radius=door_glass_r + 0.011, length=0.012),
        origin=Origin(
            xyz=(-0.020, door_outer_r, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=smoked_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.030, 0.040, 0.180)),
        origin=Origin(xyz=(0.006, 0.472, 0.0)),
        material=brushed_steel,
        name="door_pull",
    )
    door.visual(
        Cylinder(radius=0.013, length=0.028),
        origin=Origin(
            xyz=(0.018, 0.492, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brushed_steel,
        name="door_pull_cap",
    )
    for hinge_name, hinge_z in (("upper", 0.160), ("lower", -0.160)):
        door.visual(
            Cylinder(radius=0.016, length=0.074),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=brushed_steel,
            name=f"{hinge_name}_barrel",
        )
        door.visual(
            Box((0.040, 0.064, 0.084)),
            origin=Origin(xyz=(0.008, 0.026, hinge_z)),
            material=brushed_steel,
            name=f"{hinge_name}_strap",
        )

    gas_panel = model.part("gas_access_panel")
    gas_panel.inertial = Inertial.from_geometry(
        Box((gas_panel_t, gas_panel_w, gas_panel_h)),
        mass=0.6,
        origin=Origin(xyz=(-(gas_panel_t * 0.5), 0.0, gas_panel_h * 0.5)),
    )
    gas_panel.visual(
        Box((gas_panel_t, gas_panel_w, gas_panel_h)),
        origin=Origin(xyz=(-0.004, 0.0, gas_panel_h * 0.5)),
        material=stainless,
        name="service_panel",
    )
    gas_panel.visual(
        Box((gas_panel_t * 0.8, gas_panel_w - 0.026, gas_panel_h - 0.026)),
        origin=Origin(xyz=(-0.007, 0.0, gas_panel_h * 0.5)),
        material=brushed_steel,
        name="service_panel_stiffener",
    )
    gas_panel.visual(
        Cylinder(radius=0.008, length=gas_panel_w * 0.70),
        origin=Origin(
            xyz=(-0.005, 0.0, 0.008),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="service_hinge",
    )
    gas_panel.visual(
        Box((0.010, 0.050, 0.015)),
        origin=Origin(xyz=(-0.016, 0.0, gas_panel_h - 0.016)),
        material=dark_trim,
        name="service_pull",
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(drum_center_x, 0.0, door_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )
    model.articulation(
        "body_to_gas_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=gas_panel,
        origin=Origin(xyz=(rear_x - 0.003, gas_panel_y, gas_panel_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    gas_panel = object_model.get_part("gas_access_panel")

    drum_spin = object_model.get_articulation("body_to_drum")
    door_hinge = object_model.get_articulation("body_to_door")
    gas_panel_hinge = object_model.get_articulation("body_to_gas_panel")

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

    ctx.check(
        "drum spins on a continuous front-back axle",
        drum_spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(drum_spin.axis[0]) > 0.99,
        details=f"type={drum_spin.articulation_type}, axis={drum_spin.axis}",
    )
    ctx.check(
        "door hinge is vertical on the left edge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(door_hinge.axis[2]) > 0.99,
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}",
    )
    ctx.check(
        "gas access panel hinges along its lower rear edge",
        gas_panel_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(gas_panel_hinge.axis[1]) > 0.99,
        details=f"type={gas_panel_hinge.articulation_type}, axis={gas_panel_hinge.axis}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem="door_frame",
            negative_elem="front_panel",
            max_gap=0.035,
            max_penetration=0.0,
            name="door closes near-flush to the front frame",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            elem_a="door_frame",
            elem_b="front_panel",
            min_overlap=0.42,
            name="door covers the front opening footprint",
        )
        ctx.expect_overlap(
            drum,
            door,
            axes="yz",
            elem_a="drum_shell",
            elem_b="door_glass",
            min_overlap=0.34,
            name="door window is centered over the drum",
        )

    with ctx.pose({gas_panel_hinge: 0.0}):
        ctx.expect_gap(
            body,
            gas_panel,
            axis="x",
            positive_elem="rear_panel",
            negative_elem="service_panel",
            max_gap=0.020,
            max_penetration=0.0,
            name="gas panel closes near-flush to the rear face",
        )
        ctx.expect_overlap(
            gas_panel,
            body,
            axes="yz",
            elem_a="service_panel",
            elem_b="rear_panel",
            min_overlap=0.12,
            name="gas panel covers the rear access opening footprint",
        )

    def _center_of(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    with ctx.pose({door_hinge: 0.0}):
        door_closed = _center_of(ctx.part_element_world_aabb(door, elem="door_frame"))
    with ctx.pose({door_hinge: math.radians(100.0)}):
        door_open = _center_of(ctx.part_element_world_aabb(door, elem="door_frame"))
    ctx.check(
        "door opens outward from the cabinet",
        door_closed is not None
        and door_open is not None
        and door_open[0] > door_closed[0] + 0.12,
        details=f"closed={door_closed}, open={door_open}",
    )

    with ctx.pose({gas_panel_hinge: 0.0}):
        panel_closed = _center_of(ctx.part_element_world_aabb(gas_panel, elem="service_panel"))
    with ctx.pose({gas_panel_hinge: math.radians(85.0)}):
        panel_open = _center_of(ctx.part_element_world_aabb(gas_panel, elem="service_panel"))
    ctx.check(
        "gas panel swings downward and outward",
        panel_closed is not None
        and panel_open is not None
        and panel_open[0] < panel_closed[0] - 0.03
        and panel_open[2] < panel_closed[2] - 0.04,
        details=f"closed={panel_closed}, open={panel_open}",
    )

    with ctx.pose({door_hinge: math.radians(100.0), gas_panel_hinge: math.radians(85.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="opened door and service panel stay clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
