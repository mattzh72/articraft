from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _rect_profile(
    width: float,
    depth: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_d = depth / 2.0
    cx, cy = center
    return [
        (cx - half_w, cy - half_d),
        (cx + half_w, cy - half_d),
        (cx + half_w, cy + half_d),
        (cx - half_w, cy + half_d),
    ]


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _add_grate(
    part,
    *,
    name_prefix: str,
    x_center: float,
    y_center: float,
    width: float,
    depth: float,
    bar: float,
    height: float,
    z_center: float,
    material,
) -> None:
    side_x = width / 2.0 - bar / 2.0
    side_y = depth / 2.0 - bar / 2.0
    part.visual(
        Box((bar, depth, height)),
        origin=Origin(xyz=(x_center - side_x, y_center, z_center)),
        material=material,
        name=f"{name_prefix}_left_bar",
    )
    part.visual(
        Box((bar, depth, height)),
        origin=Origin(xyz=(x_center + side_x, y_center, z_center)),
        material=material,
        name=f"{name_prefix}_right_bar",
    )
    part.visual(
        Box((width, bar, height)),
        origin=Origin(xyz=(x_center, y_center - side_y, z_center)),
        material=material,
        name=f"{name_prefix}_rear_bar",
    )
    part.visual(
        Box((width, bar, height)),
        origin=Origin(xyz=(x_center, y_center + side_y, z_center)),
        material=material,
        name=f"{name_prefix}_front_bar",
    )
    part.visual(
        Box((bar, depth - 2.0 * bar, height)),
        origin=Origin(xyz=(x_center, y_center, z_center)),
        material=material,
        name=f"{name_prefix}_center_bar",
    )
    part.visual(
        Box((width - 2.0 * bar, bar, height)),
        origin=Origin(xyz=(x_center, y_center, z_center)),
        material=material,
        name=f"{name_prefix}_cross_bar",
    )


def _add_knob(model: ArticulatedObject, cooktop, *, index: int, x: float, y: float, z: float, material, accent):
    knob = model.part(f"knob_{index}")
    knob.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="mount_boss",
    )
    knob.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="knob_skirt",
    )
    knob.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="knob_body",
    )
    knob.visual(
        Box((0.006, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.019, 0.013)),
        material=accent,
        name="indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.022),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        f"knob_{index}_rotation",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child=knob,
        origin=Origin(xyz=(x, y, z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gas_cooktop_with_base_cabinet", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.11, 0.11, 0.12, 1.0))
    stone = model.material("countertop_stone", rgba=(0.26, 0.27, 0.29, 1.0))
    cabinet_paint = model.material("cabinet_paint", rgba=(0.89, 0.90, 0.88, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    marker_red = model.material("marker_red", rgba=(0.72, 0.18, 0.13, 1.0))

    cabinet_width = 0.80
    cabinet_depth = 0.58
    cabinet_height = 0.72
    panel_thickness = 0.018
    toe_kick_height = 0.10
    rail_depth = 0.08

    countertop_width = 0.84
    countertop_depth = 0.63
    countertop_thickness = 0.038

    cooktop_width = 0.58
    cooktop_depth = 0.48
    cooktop_top_thickness = 0.006
    cooktop_pan_width = 0.54
    cooktop_pan_depth = 0.44
    cooktop_pan_height = 0.030
    cooktop_pan_thickness = 0.003

    countertop_opening_width = 0.55
    countertop_opening_depth = 0.47
    countertop_opening_center_y = 0.070

    countertop_mesh = _save_mesh(
        "countertop_with_cutout.obj",
        ExtrudeWithHolesGeometry(
            _rect_profile(countertop_width, countertop_depth),
            [
                _rect_profile(
                    countertop_opening_width,
                    countertop_opening_depth,
                    center=(0.0, countertop_opening_center_y),
                )
            ],
            countertop_thickness,
            cap=True,
            center=False,
            closed=True,
        ),
    )

    cabinet = model.part("cabinet_body")
    cabinet.visual(
        Box((panel_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + panel_thickness / 2.0, 0.0, cabinet_height / 2.0)),
        material=cabinet_paint,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((panel_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width / 2.0 - panel_thickness / 2.0, 0.0, cabinet_height / 2.0)),
        material=cabinet_paint,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * panel_thickness, cabinet_depth - panel_thickness, panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                panel_thickness / 2.0,
                toe_kick_height + panel_thickness / 2.0,
            )
        ),
        material=cabinet_paint,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * panel_thickness, panel_thickness, cabinet_height - toe_kick_height)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth / 2.0 + panel_thickness / 2.0,
                toe_kick_height + (cabinet_height - toe_kick_height) / 2.0,
            )
        ),
        material=cabinet_paint,
        name="back_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * panel_thickness, rail_depth, panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth / 2.0 - rail_depth / 2.0,
                cabinet_height - panel_thickness / 2.0,
            )
        ),
        material=cabinet_paint,
        name="top_front_rail",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * panel_thickness, rail_depth, panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth / 2.0 + rail_depth / 2.0,
                cabinet_height - panel_thickness / 2.0,
            )
        ),
        material=cabinet_paint,
        name="top_back_rail",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * panel_thickness, panel_thickness, toe_kick_height)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth / 2.0 - 0.060,
                toe_kick_height / 2.0,
            )
        ),
        material=cabinet_paint,
        name="toe_kick_board",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height / 2.0)),
    )

    countertop = model.part("countertop")
    countertop.visual(
        countertop_mesh,
        origin=Origin(),
        material=stone,
        name="countertop_slab",
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((cooktop_width, cooktop_depth, cooktop_top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cooktop_top_thickness / 2.0)),
        material=stainless,
        name="stainless_top",
    )
    cooktop.visual(
        Box((0.50, 0.034, 0.008)),
        origin=Origin(xyz=(0.0, 0.214, 0.010)),
        material=stainless,
        name="front_fascia",
    )

    knob_pad_width = 0.060
    knob_pad_depth = 0.014
    knob_pad_height = 0.010
    knob_pad_z = 0.011
    knob_pad_data = [
        (-0.180, 0.210, "control_pad_1"),
        (-0.060, 0.218, "control_pad_2"),
        (0.060, 0.218, "control_pad_3"),
        (0.180, 0.210, "control_pad_4"),
    ]
    for x_pos, front_face_y, name in knob_pad_data:
        cooktop.visual(
            Box((knob_pad_width, knob_pad_depth, knob_pad_height)),
            origin=Origin(
                xyz=(
                    x_pos,
                    front_face_y - knob_pad_depth / 2.0,
                    knob_pad_z,
                )
            ),
            material=stainless,
            name=name,
        )

    cooktop.visual(
        Box((cooktop_pan_thickness, cooktop_pan_depth, cooktop_pan_height)),
        origin=Origin(
            xyz=(
                -cooktop_pan_width / 2.0 + cooktop_pan_thickness / 2.0,
                0.0,
                -cooktop_pan_height / 2.0,
            )
        ),
        material=stainless,
        name="pan_left_wall",
    )
    cooktop.visual(
        Box((cooktop_pan_thickness, cooktop_pan_depth, cooktop_pan_height)),
        origin=Origin(
            xyz=(
                cooktop_pan_width / 2.0 - cooktop_pan_thickness / 2.0,
                0.0,
                -cooktop_pan_height / 2.0,
            )
        ),
        material=stainless,
        name="pan_right_wall",
    )
    cooktop.visual(
        Box((cooktop_pan_width - 2.0 * cooktop_pan_thickness, cooktop_pan_thickness, cooktop_pan_height)),
        origin=Origin(
            xyz=(
                0.0,
                -cooktop_pan_depth / 2.0 + cooktop_pan_thickness / 2.0,
                -cooktop_pan_height / 2.0,
            )
        ),
        material=stainless,
        name="pan_back_wall",
    )
    cooktop.visual(
        Box((cooktop_pan_width - 2.0 * cooktop_pan_thickness, cooktop_pan_thickness, cooktop_pan_height)),
        origin=Origin(
            xyz=(
                0.0,
                cooktop_pan_depth / 2.0 - cooktop_pan_thickness / 2.0,
                -cooktop_pan_height / 2.0,
            )
        ),
        material=stainless,
        name="pan_front_wall",
    )
    cooktop.visual(
        Box(
            (
                cooktop_pan_width - 2.0 * cooktop_pan_thickness,
                cooktop_pan_depth - 2.0 * cooktop_pan_thickness,
                cooktop_pan_thickness,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -cooktop_pan_height + cooktop_pan_thickness / 2.0,
            )
        ),
        material=stainless,
        name="pan_floor",
    )

    burner_positions = [
        (-0.155, -0.105, 0.060, 0.030, "rear_left"),
        (0.155, -0.105, 0.060, 0.030, "rear_right"),
        (-0.155, 0.085, 0.070, 0.036, "front_left"),
        (0.155, 0.085, 0.070, 0.036, "front_right"),
    ]
    for burner_x, burner_y, ring_radius, cap_radius, burner_name in burner_positions:
        cooktop.visual(
            Cylinder(radius=ring_radius, length=0.004),
            origin=Origin(xyz=(burner_x, burner_y, 0.008)),
            material=dark_cast_iron,
            name=f"{burner_name}_burner_ring",
        )
        cooktop.visual(
            Cylinder(radius=cap_radius, length=0.014),
            origin=Origin(xyz=(burner_x, burner_y, 0.017)),
            material=charcoal,
            name=f"{burner_name}_burner_cap",
        )

    _add_grate(
        cooktop,
        name_prefix="left_grate",
        x_center=-0.155,
        y_center=-0.008,
        width=0.225,
        depth=0.320,
        bar=0.012,
        height=0.010,
        z_center=0.011,
        material=dark_cast_iron,
    )
    _add_grate(
        cooktop,
        name_prefix="right_grate",
        x_center=0.155,
        y_center=-0.008,
        width=0.225,
        depth=0.320,
        bar=0.012,
        height=0.010,
        z_center=0.011,
        material=dark_cast_iron,
    )
    cooktop.inertial = Inertial.from_geometry(
        Box((cooktop_width, cooktop_depth, 0.040)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    door_gap = 0.004
    side_reveal = 0.004
    door_height = 0.595
    door_thickness = 0.018
    door_bottom = 0.110
    door_width = (cabinet_width - 2.0 * side_reveal - door_gap) / 2.0

    left_door = model.part("left_door")
    left_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, door_thickness / 2.0, door_height / 2.0)),
        material=cabinet_paint,
        name="door_panel",
    )
    left_door.visual(
        Cylinder(radius=0.004, length=0.060),
        origin=Origin(xyz=(0.010, 0.006, 0.115)),
        material=handle_metal,
        name="hinge_barrel_lower",
    )
    left_door.visual(
        Cylinder(radius=0.004, length=0.060),
        origin=Origin(xyz=(0.010, 0.006, door_height - 0.115)),
        material=handle_metal,
        name="hinge_barrel_upper",
    )
    left_door.visual(
        Cylinder(radius=0.005, length=0.140),
        origin=Origin(xyz=(door_width - 0.040, door_thickness + 0.004, door_height / 2.0)),
        material=handle_metal,
        name="pull_handle",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=4.5,
        origin=Origin(xyz=(door_width / 2.0, door_thickness / 2.0, door_height / 2.0)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(-door_width / 2.0, door_thickness / 2.0, door_height / 2.0)),
        material=cabinet_paint,
        name="door_panel",
    )
    right_door.visual(
        Cylinder(radius=0.004, length=0.060),
        origin=Origin(xyz=(-0.010, 0.006, 0.115)),
        material=handle_metal,
        name="hinge_barrel_lower",
    )
    right_door.visual(
        Cylinder(radius=0.004, length=0.060),
        origin=Origin(xyz=(-0.010, 0.006, door_height - 0.115)),
        material=handle_metal,
        name="hinge_barrel_upper",
    )
    right_door.visual(
        Cylinder(radius=0.005, length=0.140),
        origin=Origin(xyz=(-door_width + 0.040, door_thickness + 0.004, door_height / 2.0)),
        material=handle_metal,
        name="pull_handle",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=4.5,
        origin=Origin(xyz=(-door_width / 2.0, door_thickness / 2.0, door_height / 2.0)),
    )

    model.articulation(
        "cabinet_to_countertop",
        ArticulationType.FIXED,
        parent=cabinet,
        child=countertop,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height)),
    )
    model.articulation(
        "countertop_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.065, countertop_thickness)),
    )
    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(
            xyz=(
                -cabinet_width / 2.0 + side_reveal,
                cabinet_depth / 2.0,
                door_bottom,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.92,
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(
            xyz=(
                cabinet_width / 2.0 - side_reveal,
                cabinet_depth / 2.0,
                door_bottom,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-1.92,
            upper=0.0,
        ),
    )

    for knob_index, (knob_x, knob_front_y, _) in enumerate(knob_pad_data, start=1):
        _add_knob(
            model,
            cooktop,
            index=knob_index,
            x=knob_x,
            y=knob_front_y + 0.010,
            z=0.034,
            material=charcoal,
            accent=marker_red,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet_body")
    countertop = object_model.get_part("countertop")
    cooktop = object_model.get_part("cooktop")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    knobs = [object_model.get_part(f"knob_{index}") for index in range(1, 5)]

    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    knob_joints = [object_model.get_articulation(f"knob_{index}_rotation") for index in range(1, 5)]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(countertop, cabinet, elem_a="countertop_slab", elem_b="top_front_rail", name="countertop_supported_by_cabinet")
    ctx.expect_gap(
        cooktop,
        countertop,
        axis="z",
        positive_elem="stainless_top",
        negative_elem="countertop_slab",
        max_gap=0.001,
        max_penetration=0.0,
        name="cooktop_lip_flush_with_countertop",
    )
    ctx.expect_within(
        cooktop,
        countertop,
        axes="xy",
        inner_elem="stainless_top",
        outer_elem="countertop_slab",
        margin=0.0,
        name="cooktop_seated_within_countertop_footprint",
    )
    ctx.expect_gap(
        left_door,
        cabinet,
        axis="y",
        positive_elem="door_panel",
        negative_elem="left_side_panel",
        max_gap=0.0,
        max_penetration=0.0,
        name="left_door_closed_flush",
    )
    ctx.expect_gap(
        right_door,
        cabinet,
        axis="y",
        positive_elem="door_panel",
        negative_elem="right_side_panel",
        max_gap=0.0,
        max_penetration=0.0,
        name="right_door_closed_flush",
    )

    for index, knob in enumerate(knobs, start=1):
        ctx.expect_gap(
            knob,
            cooktop,
            axis="y",
            positive_elem="mount_boss",
            negative_elem=f"control_pad_{index}",
            max_gap=0.0,
            max_penetration=0.0,
            name=f"knob_{index}_mounted_to_control_pad",
        )

    ctx.check(
        "left_hinge_axis_and_limits",
        left_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(left_hinge.axis) == (0.0, 0.0, 1.0)
        and left_hinge.motion_limits is not None
        and left_hinge.motion_limits.lower == 0.0
        and left_hinge.motion_limits.upper is not None
        and 1.7 <= left_hinge.motion_limits.upper <= 2.1,
        details="Left door should use a vertical revolute hinge with about 110 degrees of swing.",
    )
    ctx.check(
        "right_hinge_axis_and_limits",
        right_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(right_hinge.axis) == (0.0, 0.0, 1.0)
        and right_hinge.motion_limits is not None
        and right_hinge.motion_limits.upper == 0.0
        and right_hinge.motion_limits.lower is not None
        and -2.1 <= right_hinge.motion_limits.lower <= -1.7,
        details="Right door should use a vertical revolute hinge with mirrored opening travel.",
    )

    for index, joint in enumerate(knob_joints, start=1):
        ctx.check(
            f"{joint.name}_continuous_axis",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (0.0, 1.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None,
            details="Cooktop knobs should spin continuously around front-to-back axes.",
        )

    knob_positions = [ctx.part_world_position(knob) for knob in knobs]
    knob_arc_ok = (
        all(position is not None for position in knob_positions)
        and knob_positions[0][0] < knob_positions[1][0] < knob_positions[2][0] < knob_positions[3][0]
        and abs(knob_positions[0][1] - knob_positions[3][1]) < 0.001
        and abs(knob_positions[1][1] - knob_positions[2][1]) < 0.001
        and knob_positions[1][1] - knob_positions[0][1] > 0.005
    )
    ctx.check(
        "knobs_follow_gentle_arc",
        knob_arc_ok,
        details="The two center knobs should sit slightly farther forward than the outer pair.",
    )

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    countertop_aabb = ctx.part_world_aabb(countertop)
    overall_proportions_ok = (
        cabinet_aabb is not None
        and countertop_aabb is not None
        and 0.79 <= cabinet_aabb[1][0] - cabinet_aabb[0][0] <= 0.81
        and 0.71 <= cabinet_aabb[1][2] - cabinet_aabb[0][2] <= 0.73
        and 0.83 <= countertop_aabb[1][0] - countertop_aabb[0][0] <= 0.85
        and 0.75 <= countertop_aabb[1][2] <= 0.77
    )
    ctx.check(
        "realistic_cabinet_and_countertop_scale",
        overall_proportions_ok,
        details="Base cabinet and countertop should read at realistic kitchen dimensions.",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        left_closed = ctx.part_element_world_aabb(left_door, elem="door_panel")
        right_closed = ctx.part_element_world_aabb(right_door, elem="door_panel")
    ctx.check(
        "door_center_gap_present",
        left_closed is not None
        and right_closed is not None
        and 0.003 <= right_closed[0][0] - left_closed[1][0] <= 0.005,
        details="Closed doors should leave a narrow center reveal rather than overlap.",
    )

    for joint in (left_hinge, right_hinge):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    with ctx.pose({left_hinge: left_hinge.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="left_door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="left_door_open_no_floating")
        left_open = ctx.part_element_world_aabb(left_door, elem="door_panel")
    ctx.check(
        "left_door_swings_outward",
        left_open is not None and left_open[1][1] > 0.50,
        details="Left door should swing outward in front of the cabinet when opened.",
    )

    with ctx.pose({right_hinge: right_hinge.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="right_door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="right_door_open_no_floating")
        right_open = ctx.part_element_world_aabb(right_door, elem="door_panel")
    ctx.check(
        "right_door_swings_outward",
        right_open is not None and right_open[1][1] > 0.50,
        details="Right door should swing outward in front of the cabinet when opened.",
    )

    for index, (knob, joint) in enumerate(zip(knobs, knob_joints), start=1):
        with ctx.pose({joint: 0.0}):
            rest_aabb = ctx.part_element_world_aabb(knob, elem="indicator")
        with ctx.pose({joint: math.pi / 2.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_quarter_turn_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_quarter_turn_no_floating")
            turned_aabb = ctx.part_element_world_aabb(knob, elem="indicator")
        rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0 if rest_aabb is not None else None
        turned_center_z = (turned_aabb[0][2] + turned_aabb[1][2]) / 2.0 if turned_aabb is not None else None
        ctx.check(
            f"knob_{index}_indicator_moves_when_rotated",
            rest_center_z is not None and turned_center_z is not None and abs(rest_center_z - turned_center_z) > 0.010,
            details="Each knob should visibly rotate around its front-to-back axis.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
