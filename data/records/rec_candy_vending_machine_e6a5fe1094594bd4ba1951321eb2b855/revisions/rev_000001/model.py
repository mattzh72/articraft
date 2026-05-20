from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_quarter_candy_dispenser")

    brushed_metal = model.material("brushed_metal", rgba=(0.62, 0.63, 0.61, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.12, 0.13, 1.0))
    black = model.material("black_shadow", rgba=(0.01, 0.01, 0.01, 1.0))
    clear_acrylic = model.material("clear_acrylic", rgba=(0.70, 0.92, 1.0, 0.34))
    smoky = model.material("smoky_clear", rgba=(0.18, 0.23, 0.26, 0.55))
    red = model.material("red_enamel", rgba=(0.58, 0.05, 0.04, 1.0))
    ivory = model.material("ivory_label", rgba=(0.93, 0.86, 0.67, 1.0))

    # Root/static wall-mounted assembly.  +Y is outward from the wall/backplate,
    # X is dispenser width, and Z is height above the wall mount bottom.
    backplate = model.part("backplate")
    backplate.visual(
        Box((0.36, 0.026, 0.86)),
        origin=Origin(xyz=(0.0, -0.012, 0.43)),
        material=dark_metal,
        name="wall_backplate",
    )

    # Mechanism body and its metal front face.
    backplate.visual(
        Box((0.245, 0.132, 0.245)),
        origin=Origin(xyz=(0.0, 0.065, 0.225)),
        material=red,
        name="mechanism_body",
    )
    backplate.visual(
        Box((0.270, 0.012, 0.285)),
        origin=Origin(xyz=(0.0, 0.137, 0.230)),
        material=brushed_metal,
        name="front_plate",
    )
    for ix, sx in enumerate((-1.0, 1.0)):
        for iz, sz in enumerate((-1.0, 1.0)):
            backplate.visual(
                Cylinder(radius=0.0075, length=0.004),
                origin=Origin(
                    xyz=(sx * 0.105, 0.145, 0.230 + sz * 0.112),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark_metal,
                name=f"plate_screw_{ix}_{iz}",
            )

    # Quarter slot, price plaque, and dark retrieval-chute mouth details.
    backplate.visual(
        Box((0.014, 0.003, 0.070)),
        origin=Origin(xyz=(-0.078, 0.1445, 0.270)),
        material=black,
        name="coin_slot",
    )
    backplate.visual(
        Box((0.052, 0.003, 0.030)),
        origin=Origin(xyz=(-0.078, 0.1445, 0.335)),
        material=ivory,
        name="price_plaque",
    )
    backplate.visual(
        Box((0.166, 0.006, 0.088)),
        origin=Origin(xyz=(0.0, 0.145, 0.138)),
        material=black,
        name="chute_shadow",
    )
    backplate.visual(
        Box((0.188, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.151, 0.184)),
        material=brushed_metal,
        name="chute_top_lip",
    )
    for ix, sx in enumerate((-1.0, 1.0)):
        backplate.visual(
            Box((0.012, 0.010, 0.090)),
            origin=Origin(xyz=(sx * 0.094, 0.151, 0.138)),
            material=brushed_metal,
            name=f"chute_side_lip_{ix}",
        )

    # Bottom retrieval-flap fixed hinge knuckles: two outer barrels on the body.
    for ix, sx in enumerate((-1.0, 1.0)):
        backplate.visual(
            Box((0.045, 0.009, 0.014)),
            origin=Origin(xyz=(sx * 0.101, 0.148, 0.094)),
            material=brushed_metal,
            name=f"retrieval_hinge_leaf_{ix}",
        )
    for ix, sx in enumerate((-1.0, 1.0)):
        backplate.visual(
            Cylinder(radius=0.0075, length=0.032),
            origin=Origin(
                xyz=(sx * 0.101, 0.153, 0.096),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brushed_metal,
            name=f"retrieval_hinge_knuckle_{ix}",
        )

    # Hopper throat/collar mechanically ties the clear hopper to the mechanism.
    backplate.visual(
        Box((0.150, 0.118, 0.052)),
        origin=Origin(xyz=(0.0, 0.060, 0.350)),
        material=brushed_metal,
        name="hopper_throat",
    )
    backplate.visual(
        Box((0.190, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, 0.163, 0.373)),
        material=brushed_metal,
        name="hopper_front_sill",
    )

    # Rectangular transparent hopper: individual thin panes keep it visibly hollow.
    hopper_center_z = 0.550
    backplate.visual(
        Box((0.272, 0.006, 0.340)),
        origin=Origin(xyz=(0.0, 0.168, hopper_center_z)),
        material=clear_acrylic,
        name="hopper_front_pane",
    )
    backplate.visual(
        Box((0.007, 0.170, 0.340)),
        origin=Origin(xyz=(-0.136, 0.083, hopper_center_z)),
        material=clear_acrylic,
        name="hopper_side_pane_0",
    )
    backplate.visual(
        Box((0.014, 0.170, 0.012)),
        origin=Origin(xyz=(-0.136, 0.083, 0.726)),
        material=brushed_metal,
        name="hopper_top_side_rail_0",
    )
    backplate.visual(
        Box((0.007, 0.170, 0.340)),
        origin=Origin(xyz=(0.136, 0.083, hopper_center_z)),
        material=clear_acrylic,
        name="hopper_side_pane_1",
    )
    backplate.visual(
        Box((0.014, 0.170, 0.012)),
        origin=Origin(xyz=(0.136, 0.083, 0.726)),
        material=brushed_metal,
        name="hopper_top_side_rail_1",
    )
    backplate.visual(
        Box((0.292, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.171, 0.726)),
        material=brushed_metal,
        name="hopper_top_front_rail",
    )
    backplate.visual(
        Box((0.292, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.170, 0.374)),
        material=brushed_metal,
        name="hopper_bottom_front_rail",
    )
    for ix, sx in enumerate((-1.0, 1.0)):
        backplate.visual(
            Box((0.018, 0.020, 0.364)),
            origin=Origin(xyz=(sx * 0.148, 0.002, 0.550)),
            material=brushed_metal,
            name=f"rear_hopper_bracket_{ix}",
        )

    # Fixed rear half of the refill-lid hinge, carried just forward of the wall.
    backplate.visual(
        Box((0.250, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.006, 0.735)),
        material=brushed_metal,
        name="refill_hinge_leaf",
    )
    for ix, sx in enumerate((-1.0, 1.0)):
        backplate.visual(
            Cylinder(radius=0.006, length=0.058),
            origin=Origin(
                xyz=(sx * 0.083, 0.011, 0.735),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brushed_metal,
            name=f"refill_hinge_knuckle_{ix}",
        )

    # User-facing continuously rotating selector knob on a horizontal shaft.
    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="knob_shaft",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.095,
            0.036,
            body_style="faceted",
            base_diameter=0.100,
            top_diameter=0.082,
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0015, width=0.0022),
            indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        ),
        "selector_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.062, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="knob_cap",
    )
    backplate.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.151, 0.245), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="knob_bearing_collar",
    )
    model.articulation(
        "backplate_to_knob",
        ArticulationType.CONTINUOUS,
        parent=backplate,
        child=knob,
        origin=Origin(xyz=(0.0, 0.159, 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=7.0),
    )

    # Lower retrieval flap with its center hinge barrel.
    retrieval_flap = model.part("retrieval_flap")
    retrieval_flap.visual(
        Box((0.150, 0.008, 0.078)),
        origin=Origin(xyz=(0.0, 0.004, 0.039)),
        material=smoky,
        name="flap_panel",
    )
    retrieval_flap.visual(
        Box((0.154, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.006, 0.077)),
        material=brushed_metal,
        name="flap_pull_lip",
    )
    retrieval_flap.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="flap_hinge_barrel",
    )
    model.articulation(
        "backplate_to_retrieval_flap",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=retrieval_flap,
        origin=Origin(xyz=(0.0, 0.153, 0.096)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.25),
    )

    # Refill lid spanning the hopper top, hinged along the rear edge.
    refill_lid = model.part("refill_lid")
    refill_lid.visual(
        Box((0.300, 0.176, 0.010)),
        origin=Origin(xyz=(0.0, 0.106, 0.006)),
        material=brushed_metal,
        name="lid_panel",
    )
    refill_lid.visual(
        Box((0.285, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.188, 0.011)),
        material=dark_metal,
        name="lid_front_lip",
    )
    refill_lid.visual(
        Box((0.082, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.012, 0.004)),
        material=brushed_metal,
        name="lid_hinge_leaf",
    )
    refill_lid.visual(
        Cylinder(radius=0.005, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="lid_hinge_barrel",
    )
    model.articulation(
        "backplate_to_refill_lid",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=refill_lid,
        origin=Origin(xyz=(0.0, 0.011, 0.735)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    knob = object_model.get_part("knob")
    retrieval_flap = object_model.get_part("retrieval_flap")
    refill_lid = object_model.get_part("refill_lid")
    knob_joint = object_model.get_articulation("backplate_to_knob")
    flap_joint = object_model.get_articulation("backplate_to_retrieval_flap")
    lid_joint = object_model.get_articulation("backplate_to_refill_lid")

    ctx.check(
        "selector knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.check(
        "flap and refill lid are hinged",
        flap_joint.articulation_type == ArticulationType.REVOLUTE
        and lid_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"flap={flap_joint.articulation_type}, lid={lid_joint.articulation_type}",
    )

    required_hopper_panes = {
        "hopper_front_pane",
        "hopper_side_pane_0",
        "hopper_side_pane_1",
        "hopper_top_front_rail",
        "hopper_bottom_front_rail",
    }
    visual_names = {visual.name for visual in backplate.visuals}
    ctx.check(
        "hopper is built from clear hollow panes",
        required_hopper_panes.issubset(visual_names),
        details=f"missing={sorted(required_hopper_panes - visual_names)}",
    )

    backplate_aabb = ctx.part_element_world_aabb(backplate, elem="wall_backplate")
    ctx.check(
        "wall vending scale backplate",
        backplate_aabb is not None
        and 0.80 <= (backplate_aabb[1][2] - backplate_aabb[0][2]) <= 0.90
        and 0.32 <= (backplate_aabb[1][0] - backplate_aabb[0][0]) <= 0.40,
        details=f"backplate_aabb={backplate_aabb}",
    )

    ctx.expect_contact(
        knob,
        backplate,
        elem_a="knob_shaft",
        elem_b="knob_bearing_collar",
        contact_tol=0.0015,
        name="knob shaft seats at front bearing",
    )
    ctx.expect_overlap(
        retrieval_flap,
        backplate,
        axes="xz",
        elem_a="flap_panel",
        elem_b="chute_shadow",
        min_overlap=0.060,
        name="retrieval flap covers chute mouth",
    )
    ctx.expect_gap(
        retrieval_flap,
        backplate,
        axis="y",
        positive_elem="flap_panel",
        negative_elem="front_plate",
        min_gap=0.006,
        max_gap=0.020,
        name="retrieval flap sits proud of front plate",
    )
    ctx.expect_overlap(
        refill_lid,
        backplate,
        axes="x",
        elem_a="lid_panel",
        elem_b="hopper_front_pane",
        min_overlap=0.250,
        name="refill lid spans hopper width",
    )
    ctx.expect_overlap(
        refill_lid,
        backplate,
        axes="y",
        elem_a="lid_panel",
        elem_b="hopper_side_pane_1",
        min_overlap=0.120,
        name="refill lid spans hopper depth",
    )

    rest_flap = ctx.part_element_world_aabb(retrieval_flap, elem="flap_pull_lip")
    with ctx.pose({flap_joint: 1.0}):
        open_flap = ctx.part_element_world_aabb(retrieval_flap, elem="flap_pull_lip")
    ctx.check(
        "retrieval flap opens outward and downward",
        rest_flap is not None
        and open_flap is not None
        and open_flap[1][1] > rest_flap[1][1] + 0.030
        and open_flap[0][2] < rest_flap[0][2] - 0.010,
        details=f"rest={rest_flap}, open={open_flap}",
    )

    rest_lid = ctx.part_element_world_aabb(refill_lid, elem="lid_front_lip")
    with ctx.pose({lid_joint: 1.0}):
        open_lid = ctx.part_element_world_aabb(refill_lid, elem="lid_front_lip")
    ctx.check(
        "refill lid opens upward from rear hinge",
        rest_lid is not None
        and open_lid is not None
        and open_lid[1][2] > rest_lid[1][2] + 0.080,
        details=f"rest={rest_lid}, open={open_lid}",
    )

    return ctx.report()


object_model = build_object_model()
