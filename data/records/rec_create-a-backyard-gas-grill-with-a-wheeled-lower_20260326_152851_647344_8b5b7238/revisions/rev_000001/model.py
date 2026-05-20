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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backyard_gas_grill", assets=ASSETS)

    def save_mesh(geometry, filename: str):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))

    def lid_section(x_pos: float, crown_scale: float) -> list[tuple[float, float, float]]:
        return [
            (x_pos, 0.000, 0.024),
            (x_pos, -0.030, 0.078 * crown_scale + 0.010),
            (x_pos, -0.160, 0.128 * crown_scale + 0.014),
            (x_pos, -0.300, 0.100 * crown_scale + 0.014),
            (x_pos, -0.390, 0.050),
            (x_pos, -0.390, 0.026),
            (x_pos, -0.290, 0.040 * crown_scale + 0.010),
            (x_pos, -0.170, 0.056 * crown_scale + 0.012),
            (x_pos, -0.050, 0.042 * crown_scale + 0.010),
            (x_pos, 0.000, 0.020),
        ]

    steel_dark = model.material("steel_dark", rgba=(0.22, 0.24, 0.26, 1.0))
    painted_black = model.material("painted_black", rgba=(0.12, 0.12, 0.13, 1.0))
    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    shelf_metal = model.material("shelf_metal", rgba=(0.64, 0.66, 0.68, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    knob_plastic = model.material("knob_plastic", rgba=(0.16, 0.16, 0.17, 1.0))

    cart = model.part("cart")
    leg_size = 0.03
    leg_height = 0.56
    leg_centers = [
        (-0.285, -0.175),
        (0.285, -0.175),
        (-0.285, 0.175),
        (0.285, 0.175),
    ]
    for index, (x_pos, y_pos) in enumerate(leg_centers, start=1):
        cart.visual(
            Box((leg_size, leg_size, leg_height)),
            origin=Origin(xyz=(x_pos, y_pos, leg_height * 0.5)),
            material=painted_black,
            name=f"leg_{index}",
        )

    cart.visual(
        Box((0.57, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, -0.175, 0.575)),
        material=painted_black,
        name="top_front_rail",
    )
    cart.visual(
        Box((0.57, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.175, 0.575)),
        material=painted_black,
        name="top_rear_rail",
    )
    cart.visual(
        Box((0.03, 0.35, 0.03)),
        origin=Origin(xyz=(-0.285, 0.0, 0.575)),
        material=painted_black,
        name="top_left_rail",
    )
    cart.visual(
        Box((0.03, 0.35, 0.03)),
        origin=Origin(xyz=(0.285, 0.0, 0.575)),
        material=painted_black,
        name="top_right_rail",
    )
    cart.visual(
        Box((0.54, 0.36, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=steel_dark,
        name="lower_shelf",
    )
    cart.visual(
        Cylinder(radius=0.012, length=0.650),
        origin=Origin(xyz=(0.0, 0.175, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_black,
        name="rear_axle",
    )
    for side, x_pos in (("left", -0.325), ("right", 0.325)):
        cart.visual(
            Cylinder(radius=0.075, length=0.040),
            origin=Origin(xyz=(x_pos, 0.175, 0.075), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=tire_rubber,
            name=f"rear_wheel_{side}",
        )
    for side, x_pos in (("left", -0.285), ("right", 0.285)):
        cart.visual(
            Cylinder(radius=0.010, length=0.100),
            origin=Origin(xyz=(x_pos, -0.175, 0.100)),
            material=painted_black,
            name=f"caster_stem_{side}",
        )
        cart.visual(
            Box((0.014, 0.030, 0.030)),
            origin=Origin(xyz=(x_pos, -0.175, 0.050)),
            material=painted_black,
            name=f"caster_fork_{side}",
        )
        cart.visual(
            Cylinder(radius=0.045, length=0.020),
            origin=Origin(xyz=(x_pos, -0.175, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=tire_rubber,
            name=f"front_wheel_{side}",
        )
    cart.inertial = Inertial.from_geometry(
        Box((0.75, 0.45, 0.65)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
    )

    cookbox = model.part("cookbox")
    cookbox.visual(
        Box((0.60, 0.38, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=painted_black,
        name="bottom_pan",
    )
    cookbox.visual(
        Box((0.02, 0.40, 0.18)),
        origin=Origin(xyz=(-0.300, 0.0, 0.100)),
        material=painted_black,
        name="left_wall",
    )
    cookbox.visual(
        Box((0.02, 0.40, 0.18)),
        origin=Origin(xyz=(0.300, 0.0, 0.100)),
        material=painted_black,
        name="right_wall",
    )
    cookbox.visual(
        Box((0.58, 0.02, 0.18)),
        origin=Origin(xyz=(0.0, -0.190, 0.100)),
        material=painted_black,
        name="front_wall",
    )
    cookbox.visual(
        Box((0.58, 0.02, 0.18)),
        origin=Origin(xyz=(0.0, 0.190, 0.100)),
        material=painted_black,
        name="rear_wall",
    )
    for index, y_pos in enumerate((-0.120, 0.0, 0.120), start=1):
        cookbox.visual(
            Box((0.58, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, y_pos, 0.115)),
            material=stainless,
            name=f"grate_bar_{index}",
        )
    for x_pos in (-0.250, 0.250):
        for y_pos in (-0.165, 0.165):
            cookbox.visual(
                Box((0.06, 0.06, 0.04)),
                origin=Origin(xyz=(x_pos, y_pos, -0.020)),
                material=painted_black,
                name=f"mount_pad_{'l' if x_pos < 0.0 else 'r'}_{'f' if y_pos < 0.0 else 'r'}",
            )
    cookbox.inertial = Inertial.from_geometry(
        Box((0.62, 0.42, 0.23)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.56, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, -0.020, -0.010)),
        material=painted_black,
        name="panel_flange",
    )
    control_panel.visual(
        Box((0.56, 0.03, 0.13)),
        origin=Origin(xyz=(0.0, -0.045, -0.075)),
        material=stainless,
        name="panel_face",
    )
    control_panel.visual(
        Box((0.03, 0.05, 0.13)),
        origin=Origin(xyz=(-0.265, -0.025, -0.075)),
        material=painted_black,
        name="left_cheek",
    )
    control_panel.visual(
        Box((0.03, 0.05, 0.13)),
        origin=Origin(xyz=(0.265, -0.025, -0.075)),
        material=painted_black,
        name="right_cheek",
    )
    for index, x_pos in enumerate((-0.180, -0.060, 0.060, 0.180), start=1):
        control_panel.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(xyz=(x_pos, -0.075, -0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_plastic,
            name=f"knob_stem_{index}",
        )
        control_panel.visual(
            Cylinder(radius=0.022, length=0.026),
            origin=Origin(xyz=(x_pos, -0.103, -0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_plastic,
            name=f"knob_{index}",
        )
    control_panel.inertial = Inertial.from_geometry(
        Box((0.58, 0.14, 0.15)),
        mass=2.0,
        origin=Origin(xyz=(0.0, -0.060, -0.070)),
    )

    left_shelf = model.part("left_shelf")
    left_shelf.visual(
        Box((0.22, 0.35, 0.02)),
        origin=Origin(xyz=(-0.110, 0.0, 0.0)),
        material=shelf_metal,
        name="shelf_top",
    )
    left_shelf.visual(
        Box((0.22, 0.02, 0.036)),
        origin=Origin(xyz=(-0.110, -0.165, 0.018)),
        material=stainless,
        name="front_rail",
    )
    left_shelf.visual(
        Box((0.22, 0.02, 0.036)),
        origin=Origin(xyz=(-0.110, 0.165, 0.018)),
        material=stainless,
        name="rear_rail",
    )
    for index, y_pos in enumerate((-0.110, 0.110), start=1):
        left_shelf.visual(
            Box((0.03, 0.04, 0.10)),
            origin=Origin(xyz=(-0.015, y_pos, -0.050)),
            material=painted_black,
            name=f"brace_{index}",
        )
    left_shelf.inertial = Inertial.from_geometry(
        Box((0.24, 0.36, 0.12)),
        mass=1.6,
        origin=Origin(xyz=(-0.120, 0.0, -0.020)),
    )

    right_shelf = model.part("right_shelf")
    right_shelf.visual(
        Box((0.22, 0.35, 0.02)),
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        material=shelf_metal,
        name="shelf_top",
    )
    right_shelf.visual(
        Box((0.22, 0.02, 0.036)),
        origin=Origin(xyz=(0.110, -0.165, 0.018)),
        material=stainless,
        name="front_rail",
    )
    right_shelf.visual(
        Box((0.22, 0.02, 0.036)),
        origin=Origin(xyz=(0.110, 0.165, 0.018)),
        material=stainless,
        name="rear_rail",
    )
    for index, y_pos in enumerate((-0.110, 0.110), start=1):
        right_shelf.visual(
            Box((0.03, 0.04, 0.10)),
            origin=Origin(xyz=(0.015, y_pos, -0.050)),
            material=painted_black,
            name=f"brace_{index}",
        )
    right_shelf.inertial = Inertial.from_geometry(
        Box((0.24, 0.36, 0.12)),
        mass=1.6,
        origin=Origin(xyz=(0.120, 0.0, -0.020)),
    )

    lid = model.part("lid")
    lid_mesh = section_loft(
        [
            lid_section(-0.310, 0.86),
            lid_section(-0.180, 0.94),
            lid_section(0.000, 1.00),
            lid_section(0.180, 0.94),
            lid_section(0.310, 0.86),
        ]
    )
    lid.visual(
        save_mesh(lid_mesh, "grill_lid.obj"),
        material=painted_black,
        name="lid_crown",
    )
    lid.visual(
        Box((0.62, 0.02, 0.09)),
        origin=Origin(xyz=(0.0, -0.390, 0.045)),
        material=painted_black,
        name="front_skirt",
    )
    lid.visual(
        Box((0.62, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, -0.010, 0.030)),
        material=painted_black,
        name="rear_skirt",
    )
    lid.visual(
        Box((0.02, 0.38, 0.09)),
        origin=Origin(xyz=(-0.300, -0.190, 0.045)),
        material=painted_black,
        name="left_skirt",
    )
    lid.visual(
        Box((0.02, 0.38, 0.09)),
        origin=Origin(xyz=(0.300, -0.190, 0.045)),
        material=painted_black,
        name="right_skirt",
    )
    for side, x_pos in (("left", -0.160), ("right", 0.160)):
        lid.visual(
            Cylinder(radius=0.010, length=0.045),
            origin=Origin(xyz=(x_pos, -0.3225, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"handle_post_{side}",
        )
    lid.visual(
        Cylinder(radius=0.011, length=0.320),
        origin=Origin(xyz=(0.0, -0.345, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="handle_bar",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.66, 0.42, 0.18)),
        mass=5.0,
        origin=Origin(xyz=(0.0, -0.180, 0.090)),
    )

    model.articulation(
        "cart_to_cookbox",
        ArticulationType.FIXED,
        parent=cart,
        child=cookbox,
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
    )
    model.articulation(
        "cookbox_to_control_panel",
        ArticulationType.FIXED,
        parent=cookbox,
        child=control_panel,
        origin=Origin(xyz=(0.0, -0.200, 0.010)),
    )
    model.articulation(
        "cookbox_to_left_shelf",
        ArticulationType.FIXED,
        parent=cookbox,
        child=left_shelf,
        origin=Origin(xyz=(-0.310, 0.0, 0.130)),
    )
    model.articulation(
        "cookbox_to_right_shelf",
        ArticulationType.FIXED,
        parent=cookbox,
        child=right_shelf,
        origin=Origin(xyz=(0.310, 0.0, 0.130)),
    )
    model.articulation(
        "cookbox_to_lid",
        ArticulationType.REVOLUTE,
        parent=cookbox,
        child=lid,
        origin=Origin(xyz=(0.0, 0.200, 0.190)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.6,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cart = object_model.get_part("cart")
    cookbox = object_model.get_part("cookbox")
    control_panel = object_model.get_part("control_panel")
    left_shelf = object_model.get_part("left_shelf")
    right_shelf = object_model.get_part("right_shelf")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("cookbox_to_lid")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    for part_name in ("cart", "cookbox", "control_panel", "left_shelf", "right_shelf", "lid"):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    ctx.expect_contact(cookbox, cart, name="cookbox mounted on cart")
    ctx.expect_contact(control_panel, cookbox, name="control panel mounted to cookbox")
    ctx.expect_contact(left_shelf, cookbox, name="left shelf attached")
    ctx.expect_contact(right_shelf, cookbox, name="right shelf attached")

    cookbox_pos = ctx.part_world_position(cookbox)
    control_pos = ctx.part_world_position(control_panel)
    left_pos = ctx.part_world_position(left_shelf)
    right_pos = ctx.part_world_position(right_shelf)
    assert cookbox_pos is not None
    assert control_pos is not None
    assert left_pos is not None
    assert right_pos is not None

    ctx.check(
        "control panel sits forward of cookbox",
        control_pos[1] < cookbox_pos[1] - 0.18,
        details=f"control_panel_y={control_pos[1]:.3f}, cookbox_y={cookbox_pos[1]:.3f}",
    )
    ctx.check(
        "side shelves flank cookbox",
        left_pos[0] < cookbox_pos[0] - 0.25 and right_pos[0] > cookbox_pos[0] + 0.25,
        details=(
            f"left_x={left_pos[0]:.3f}, cookbox_x={cookbox_pos[0]:.3f}, "
            f"right_x={right_pos[0]:.3f}"
        ),
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            cookbox,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid seats on cookbox rim",
        )
        ctx.expect_overlap(
            lid,
            cookbox,
            axes="xy",
            min_overlap=0.26,
            name="closed lid covers cookbox opening",
        )

    lid_rest_aabb = ctx.part_world_aabb(lid)
    assert lid_rest_aabb is not None
    with ctx.pose({lid_hinge: 1.10}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        assert lid_open_aabb is not None
        ctx.expect_gap(
            lid,
            cookbox,
            axis="z",
            min_gap=0.08,
            positive_elem="front_skirt",
            name="open lid front lifts above cookbox",
        )
        ctx.check(
            "lid rises when opened",
            lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.14,
            details=(
                f"rest_max_z={lid_rest_aabb[1][2]:.3f}, "
                f"open_max_z={lid_open_aabb[1][2]:.3f}"
            ),
        )
        ctx.check(
            "lid swings rearward when opened",
            lid_open_aabb[0][1] > lid_rest_aabb[0][1] + 0.12,
            details=(
                f"rest_min_y={lid_rest_aabb[0][1]:.3f}, "
                f"open_min_y={lid_open_aabb[0][1]:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
