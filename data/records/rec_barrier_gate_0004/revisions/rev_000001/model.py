from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)

ASSETS = AssetContext.from_script(__file__)

HINGE_AXIS_X = 0.0
POST_CENTER_X = -0.06
POST_SIZE = 0.05
HINGE_RADIUS = 0.018
LEAF_KNUCKLE_RADIUS = 0.016
LEAF_PANEL_START_X = 0.055
LEAF_PANEL_WIDTH = 0.81
LEAF_PANEL_THICKNESS = 0.03
LEAF_PANEL_HEIGHT = 0.95
LEAF_PANEL_BOTTOM_Z = 0.11
LEAF_PANEL_CENTER_X = LEAF_PANEL_START_X + LEAF_PANEL_WIDTH * 0.5
LEAF_PANEL_CENTER_Z = LEAF_PANEL_BOTTOM_Z + LEAF_PANEL_HEIGHT * 0.5
STOP_POST_CENTER_X = 0.94
STOP_POST_SIZE = 0.05
GATE_JOINT_Z = 0.24


def _add_support_hinge(part, *, start_z: float, steel) -> dict[str, object]:
    support_bottom = part.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.06),
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, start_z + 0.03)),
        material=steel,
        name=f"support_knuckle_{int(round(start_z * 1000)):03d}_bottom",
    )
    support_top = part.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.06),
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, start_z + 0.164)),
        material=steel,
        name=f"support_knuckle_{int(round(start_z * 1000)):03d}_top",
    )
    part.visual(
        Box((0.017, 0.020, 0.06)),
        origin=Origin(xyz=(-0.0265, 0.0, start_z + 0.03)),
        material=steel,
        name=f"support_mount_{int(round(start_z * 1000)):03d}_bottom",
    )
    part.visual(
        Box((0.017, 0.020, 0.06)),
        origin=Origin(xyz=(-0.0265, 0.0, start_z + 0.164)),
        material=steel,
        name=f"support_mount_{int(round(start_z * 1000)):03d}_top",
    )
    pin_bottom = part.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, start_z - 0.009)),
        material=steel,
        name=f"pin_end_{int(round(start_z * 1000)):03d}_bottom",
    )
    pin_top = part.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, start_z + 0.203)),
        material=steel,
        name=f"pin_end_{int(round(start_z * 1000)):03d}_top",
    )
    return {
        "support_bottom": support_bottom,
        "support_top": support_top,
        "pin_bottom": pin_bottom,
        "pin_top": pin_top,
    }


def _add_leaf_hinge(part, *, start_z: float, steel, name: str):
    return part.visual(
        Cylinder(radius=LEAF_KNUCKLE_RADIUS, length=0.074),
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, start_z + 0.097 - GATE_JOINT_Z)),
        material=steel,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestrian_swing_gate", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    galvanized = model.material("galvanized", rgba=(0.68, 0.70, 0.72, 1.0))

    support = model.part("support_assembly")
    support.visual(
        Box((1.14, 0.18, 0.025)),
        origin=Origin(xyz=(0.435, 0.0, 0.0125)),
        material=dark_steel,
        name="base_flange",
    )
    support.visual(
        Box((POST_SIZE, POST_SIZE, 1.15)),
        origin=Origin(xyz=(POST_CENTER_X, 0.0, 0.6)),
        material=dark_steel,
        name="hinge_post",
    )
    support.visual(
        Box((STOP_POST_SIZE, STOP_POST_SIZE, 1.08)),
        origin=Origin(xyz=(STOP_POST_CENTER_X, 0.0, 0.565)),
        material=dark_steel,
        name="stop_post",
    )

    lower_support_hinge = _add_support_hinge(
        support,
        start_z=0.18,
        steel=brushed_steel,
    )
    upper_support_hinge = _add_support_hinge(
        support,
        start_z=0.79,
        steel=brushed_steel,
    )

    catch_back = support.visual(
        Box((0.018, 0.042, 0.18)),
        origin=Origin(xyz=(0.902, 0.0, 0.58)),
        material=galvanized,
        name="catch_back",
    )
    support.visual(
        Box((0.005, 0.042, 0.18)),
        origin=Origin(xyz=(0.9135, 0.0, 0.58)),
        material=galvanized,
        name="catch_mount_plate",
    )
    catch_upper = support.visual(
        Box((0.035, 0.008, 0.18)),
        origin=Origin(xyz=(0.8755, 0.014, 0.58)),
        material=galvanized,
        name="catch_upper_guide",
    )
    catch_lower = support.visual(
        Box((0.035, 0.008, 0.18)),
        origin=Origin(xyz=(0.8755, -0.014, 0.58)),
        material=galvanized,
        name="catch_lower_guide",
    )
    support.visual(
        Box((0.022, 0.030, 0.050)),
        origin=Origin(xyz=(0.902, 0.0, 0.695)),
        material=galvanized,
        name="spring_case",
    )
    support.inertial = Inertial.from_geometry(
        Box((1.14, 0.18, 1.15)),
        mass=32.0,
        origin=Origin(xyz=(0.435, 0.0, 0.575)),
    )

    leaf = model.part("gate_leaf")
    panel_body = leaf.visual(
        Box((LEAF_PANEL_WIDTH, LEAF_PANEL_THICKNESS, LEAF_PANEL_HEIGHT)),
        origin=Origin(xyz=(LEAF_PANEL_CENTER_X, 0.0, LEAF_PANEL_CENTER_Z - GATE_JOINT_Z)),
        material=dark_steel,
        name="panel_body",
    )
    lower_leaf_knuckle = _add_leaf_hinge(
        leaf,
        start_z=0.18,
        steel=brushed_steel,
        name="leaf_knuckle_lower",
    )
    upper_leaf_knuckle = _add_leaf_hinge(
        leaf,
        start_z=0.79,
        steel=brushed_steel,
        name="leaf_knuckle_upper",
    )
    leaf.visual(
        Box((0.039, 0.012, 0.074)),
        origin=Origin(xyz=(0.0355, 0.0, 0.277 - GATE_JOINT_Z)),
        material=brushed_steel,
        name="leaf_hinge_strap_lower",
    )
    leaf.visual(
        Box((0.039, 0.012, 0.074)),
        origin=Origin(xyz=(0.0355, 0.0, 0.887 - GATE_JOINT_Z)),
        material=brushed_steel,
        name="leaf_hinge_strap_upper",
    )
    push_rail = leaf.visual(
        Box((0.69, 0.03, 0.07)),
        origin=Origin(xyz=(0.47, 0.03, 0.60 - GATE_JOINT_Z)),
        material=galvanized,
        name="push_rail",
    )
    latch_plate = leaf.visual(
        Box((0.028, 0.012, 0.26)),
        origin=Origin(xyz=(0.879, 0.0, 0.58 - GATE_JOINT_Z)),
        material=galvanized,
        name="latch_plate",
    )
    leaf.inertial = Inertial.from_geometry(
        Box((0.91, 0.06, 0.95)),
        mass=22.0,
        origin=Origin(xyz=(0.455, 0.0, 0.585)),
    )

    model.articulation(
        "gate_swing",
        ArticulationType.REVOLUTE,
        parent=support,
        child=leaf,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, GATE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support = object_model.get_part("support_assembly")
    leaf = object_model.get_part("gate_leaf")
    swing = object_model.get_articulation("gate_swing")

    panel_body = leaf.get_visual("panel_body")
    push_rail = leaf.get_visual("push_rail")
    latch_plate = leaf.get_visual("latch_plate")
    lower_leaf_knuckle = leaf.get_visual("leaf_knuckle_lower")
    upper_leaf_knuckle = leaf.get_visual("leaf_knuckle_upper")

    lower_support_bottom = support.get_visual("support_knuckle_180_bottom")
    lower_support_top = support.get_visual("support_knuckle_180_top")
    lower_pin_bottom = support.get_visual("pin_end_180_bottom")
    lower_pin_top = support.get_visual("pin_end_180_top")
    upper_support_bottom = support.get_visual("support_knuckle_790_bottom")
    upper_support_top = support.get_visual("support_knuckle_790_top")
    upper_pin_bottom = support.get_visual("pin_end_790_bottom")
    upper_pin_top = support.get_visual("pin_end_790_top")
    base_flange = support.get_visual("base_flange")
    hinge_post = support.get_visual("hinge_post")
    catch_back = support.get_visual("catch_back")
    catch_mount_plate = support.get_visual("catch_mount_plate")
    catch_upper = support.get_visual("catch_upper_guide")
    catch_lower = support.get_visual("catch_lower_guide")
    spring_case = support.get_visual("spring_case")
    stop_post = support.get_visual("stop_post")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(
        leaf,
        support,
        elem_a=lower_leaf_knuckle,
        elem_b=lower_support_bottom,
    )
    ctx.expect_contact(
        leaf,
        support,
        elem_a=lower_leaf_knuckle,
        elem_b=lower_support_top,
    )
    ctx.expect_contact(
        leaf,
        support,
        elem_a=upper_leaf_knuckle,
        elem_b=upper_support_bottom,
    )
    ctx.expect_contact(
        leaf,
        support,
        elem_a=upper_leaf_knuckle,
        elem_b=upper_support_top,
    )

    ctx.expect_gap(
        leaf,
        leaf,
        axis="x",
        min_gap=0.02,
        positive_elem=panel_body,
        negative_elem=lower_leaf_knuckle,
        name="hinge_barrel_stands_clear_of_panel",
    )
    ctx.expect_gap(
        leaf,
        leaf,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=push_rail,
        negative_elem=panel_body,
        name="push_rail_is_seated_on_leaf_face",
    )
    ctx.expect_within(
        leaf,
        leaf,
        axes="xz",
        inner_elem=push_rail,
        outer_elem=panel_body,
        name="push_rail_spans_leaf_face",
    )
    ctx.expect_gap(
        leaf,
        leaf,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=latch_plate,
        negative_elem=panel_body,
        name="latch_plate_mounts_to_free_edge",
    )

    ctx.expect_overlap(
        leaf,
        support,
        axes="z",
        min_overlap=0.17,
        elem_a=latch_plate,
        elem_b=catch_back,
        name="latch_plate_aligns_with_catch",
    )
    ctx.expect_overlap(
        leaf,
        support,
        axes="y",
        min_overlap=0.01,
        elem_a=latch_plate,
        elem_b=catch_back,
        name="latch_plate_centers_on_catch",
    )
    ctx.expect_gap(
        support,
        leaf,
        axis="x",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem=catch_back,
        negative_elem=latch_plate,
        name="latch_plate_reaches_catch_back",
    )
    ctx.expect_gap(
        support,
        leaf,
        axis="y",
        max_gap=0.005,
        max_penetration=0.0,
        positive_elem=catch_upper,
        negative_elem=latch_plate,
        name="latch_plate_fits_below_upper_catch_guide",
    )
    ctx.expect_gap(
        leaf,
        support,
        axis="y",
        max_gap=0.005,
        max_penetration=0.0,
        positive_elem=latch_plate,
        negative_elem=catch_lower,
        name="latch_plate_fits_above_lower_catch_guide",
    )

    ctx.expect_gap(
        support,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lower_pin_top,
        negative_elem=lower_support_top,
        name="lower_top_pin_end_is_visible",
    )
    ctx.expect_gap(
        support,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lower_support_bottom,
        negative_elem=lower_pin_bottom,
        name="lower_bottom_pin_end_is_visible",
    )
    ctx.expect_gap(
        support,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=upper_pin_top,
        negative_elem=upper_support_top,
        name="upper_top_pin_end_is_visible",
    )
    ctx.expect_gap(
        support,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=upper_support_bottom,
        negative_elem=upper_pin_bottom,
        name="upper_bottom_pin_end_is_visible",
    )
    ctx.expect_gap(
        support,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=hinge_post,
        negative_elem=base_flange,
        name="hinge_post_is_set_on_base_flange",
    )
    ctx.expect_contact(
        support,
        support,
        elem_a=stop_post,
        elem_b=base_flange,
    )
    ctx.expect_contact(
        support,
        support,
        elem_a=stop_post,
        elem_b=catch_mount_plate,
    )
    ctx.expect_gap(
        support,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=spring_case,
        negative_elem=catch_back,
        name="spring_case_caps_catch_body",
    )

    with ctx.pose({swing: 1.2}):
        ctx.expect_gap(
            leaf,
            support,
            axis="y",
            min_gap=0.40,
            positive_elem=latch_plate,
            negative_elem=stop_post,
            name="open_gate_swings_clear_of_stop_post",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
