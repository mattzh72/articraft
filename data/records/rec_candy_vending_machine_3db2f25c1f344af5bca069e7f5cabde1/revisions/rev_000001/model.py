from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _globe_shell_mesh():
    """Thin transparent spherical shell with real top and bottom openings."""

    radius = 0.30
    wall = 0.008
    z_min = -0.248
    z_max = 0.248
    steps = 18

    outer = []
    inner = []
    for i in range(steps + 1):
        z = z_min + (z_max - z_min) * i / steps
        r = math.sqrt(max(radius * radius - z * z, 0.0))
        outer.append((r, z))

        # Offset radially inward enough to read as a hollow acrylic globe.
        scale = max(radius - wall, 0.0) / radius
        inner_z = z * scale
        inner_r = math.sqrt(max((radius - wall) ** 2 - inner_z * inner_z, 0.0))
        inner.append((inner_r, inner_z))

    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_gumball_machine")

    red = model.material("red_enamel", rgba=(0.72, 0.04, 0.025, 1.0))
    dark_red = model.material("dark_red_shadow", rgba=(0.35, 0.012, 0.01, 1.0))
    chrome = model.material("cast_chrome", rgba=(0.72, 0.70, 0.65, 1.0))
    dark = model.material("black_recess", rgba=(0.025, 0.022, 0.020, 1.0))
    brass = model.material("brass_pin", rgba=(0.95, 0.66, 0.22, 1.0))
    clear = model.material("clear_acrylic", rgba=(0.72, 0.93, 1.0, 0.30))
    candy_red = model.material("candy_red", rgba=(0.95, 0.02, 0.03, 1.0))
    candy_yellow = model.material("candy_yellow", rgba=(1.0, 0.84, 0.05, 1.0))
    candy_blue = model.material("candy_blue", rgba=(0.05, 0.22, 0.95, 1.0))
    candy_green = model.material("candy_green", rgba=(0.05, 0.75, 0.18, 1.0))
    candy_white = model.material("candy_white", rgba=(0.96, 0.94, 0.88, 1.0))

    body = model.part("body")

    # Lobby-scale pedestal and red cast housing.
    body.visual(
        Cylinder(radius=0.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=red,
        name="floor_base",
    )
    body.visual(
        Cylinder(radius=0.19, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=dark_red,
        name="base_step",
    )
    body.visual(
        Cylinder(radius=0.065, length=0.41),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=red,
        name="pedestal_column",
    )
    body.visual(
        Cylinder(radius=0.105, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=red,
        name="column_cap",
    )
    body.visual(
        Box((0.34, 0.42, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=red,
        name="mechanism_body",
    )
    body.visual(
        Cylinder(radius=0.17, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.835)),
        material=red,
        name="globe_saddle",
    )
    body.visual(
        Cylinder(radius=0.115, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.885)),
        material=red,
        name="short_neck",
    )

    # Clear globe, open at crown and at the neck.
    body.visual(
        mesh_from_geometry(_globe_shell_mesh(), "open_clear_globe"),
        origin=Origin(xyz=(0.0, 0.0, 1.150)),
        material=clear,
        name="open_clear_globe",
    )
    body.visual(
        Cylinder(radius=0.188, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.908)),
        material=chrome,
        name="lower_globe_clamp",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(0.174, 0.006, radial_segments=72, tubular_segments=12), "top_opening_rim"),
        origin=Origin(xyz=(0.0, 0.0, 1.400)),
        material=chrome,
        name="top_opening_rim",
    )

    # A settled pile of gumballs inside the clear globe.  The balls touch one
    # another and the lower neck so they read as retained contents rather than
    # unsupported decorations.
    gumballs = [
        ((-0.070, -0.040, 0.948), candy_red),
        ((0.005, -0.052, 0.950), candy_yellow),
        ((0.078, -0.035, 0.949), candy_blue),
        ((-0.036, 0.034, 0.952), candy_green),
        ((0.044, 0.038, 0.954), candy_white),
        ((-0.088, 0.022, 1.020), candy_blue),
        ((-0.012, -0.010, 1.022), candy_red),
        ((0.067, 0.002, 1.020), candy_yellow),
        ((-0.043, 0.065, 1.025), candy_green),
        ((0.030, 0.070, 1.030), candy_red),
        ((-0.004, 0.020, 1.092), candy_blue),
    ]
    for index, (xyz, material) in enumerate(gumballs):
        body.visual(
            Sphere(radius=0.046),
            origin=Origin(xyz=xyz),
            material=material,
            name=f"gumball_{index}",
        )

    # Cast-metal coin mechanism on the front (+X) face.
    body.visual(
        Box((0.026, 0.275, 0.255)),
        origin=Origin(xyz=(0.183, 0.0, 0.675)),
        material=chrome,
        name="coin_plate",
    )
    body.visual(
        Box((0.010, 0.225, 0.018)),
        origin=Origin(xyz=(0.201, 0.0, 0.795)),
        material=chrome,
        name="coin_plate_top_lip",
    )
    body.visual(
        Box((0.011, 0.122, 0.012)),
        origin=Origin(xyz=(0.200, 0.0, 0.748)),
        material=dark,
        name="coin_slot",
    )
    body.visual(
        Box((0.012, 0.105, 0.032)),
        origin=Origin(xyz=(0.202, 0.0, 0.708)),
        material=chrome,
        name="coin_label_pad",
    )
    body.visual(
        Box((0.012, 0.040, 0.020)),
        origin=Origin(xyz=(0.209, 0.0, 0.708)),
        material=dark,
        name="five_cent_mark",
    )
    body.visual(
        Cylinder(radius=0.046, length=0.030),
        origin=Origin(xyz=(0.193, 0.0, 0.640), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="knob_bushing",
    )
    for y in (-0.108, 0.108):
        for z in (0.575, 0.790):
            body.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(xyz=(0.200, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark,
                name=f"plate_screw_{y:+.3f}_{z:.3f}",
            )

    # Lower retrieval chute and stationary hinge leaves.
    body.visual(
        Box((0.014, 0.245, 0.150)),
        origin=Origin(xyz=(0.178, 0.0, 0.535)),
        material=chrome,
        name="chute_frame",
    )
    body.visual(
        Box((0.010, 0.195, 0.102)),
        origin=Origin(xyz=(0.186, 0.0, 0.535)),
        material=dark,
        name="chute_recess",
    )
    for y in (-0.125, 0.125):
        body.visual(
            Box((0.034, 0.036, 0.030)),
            origin=Origin(xyz=(0.188, y, 0.475)),
            material=chrome,
            name=f"chute_hinge_leaf_{y:+.3f}",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.034),
            origin=Origin(xyz=(0.206, y, 0.475), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"chute_hinge_knuckle_{y:+.3f}",
        )

    # Rear crown hinge mounts for the top refill lid.
    for y in (-0.095, 0.095):
        body.visual(
            Box((0.040, 0.050, 0.012)),
            origin=Origin(xyz=(-0.181, y, 1.392)),
            material=chrome,
            name=f"lid_hinge_bridge_{y:+.3f}",
        )
        body.visual(
            Box((0.060, 0.050, 0.028)),
            origin=Origin(xyz=(-0.225, y, 1.403)),
            material=chrome,
            name=f"lid_hinge_leaf_{y:+.3f}",
        )
        body.visual(
            Cylinder(radius=0.014, length=0.050),
            origin=Origin(xyz=(-0.205, y, 1.425), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"lid_hinge_knuckle_{y:+.3f}",
        )

    # Articulated refill lid: hinged at the rear of the top opening, not a
    # sealed continuation of the sphere.
    refill_lid = model.part("refill_lid")
    refill_lid.visual(
        Cylinder(radius=0.182, length=0.024),
        origin=Origin(xyz=(0.205, 0.0, -0.005)),
        material=red,
        name="lid_cap",
    )
    refill_lid.visual(
        Cylinder(radius=0.145, length=0.008),
        origin=Origin(xyz=(0.205, 0.0, 0.001)),
        material=chrome,
        name="lid_top_badge",
    )
    refill_lid.visual(
        Box((0.160, 0.070, 0.010)),
        origin=Origin(xyz=(0.080, 0.0, -0.010)),
        material=chrome,
        name="lid_hinge_leaf",
    )
    refill_lid.visual(
        Cylinder(radius=0.013, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lid_hinge_barrel",
    )

    model.articulation(
        "body_to_refill_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=refill_lid,
        origin=Origin(xyz=(-0.205, 0.0, 1.425)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    # Continuous rotating dispense knob on a horizontal shaft.
    knob = model.part("dispense_knob")
    knob.visual(
        Cylinder(radius=0.018, length=0.096),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="shaft_collar",
    )
    knob.visual(
        Cylinder(radius=0.066, length=0.034),
        origin=Origin(xyz=(0.068, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="turning_disk",
    )
    knob.visual(
        Box((0.026, 0.175, 0.030)),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material=chrome,
        name="turning_handle",
    )
    knob.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.103, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="center_button",
    )

    model.articulation(
        "body_to_dispense_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.206, 0.0, 0.640)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0),
    )

    # Chute flap hinged along the lower edge; it opens outward and down.
    chute_flap = model.part("chute_flap")
    chute_flap.visual(
        Cylinder(radius=0.011, length=0.214),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="flap_hinge_barrel",
    )
    chute_flap.visual(
        Box((0.018, 0.185, 0.110)),
        origin=Origin(xyz=(0.010, 0.0, 0.057)),
        material=chrome,
        name="flap_panel",
    )
    chute_flap.visual(
        Box((0.008, 0.080, 0.018)),
        origin=Origin(xyz=(0.023, 0.0, 0.088)),
        material=dark,
        name="finger_pull",
    )

    model.articulation(
        "body_to_chute_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=chute_flap,
        origin=Origin(xyz=(0.206, 0.0, 0.475)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("refill_lid")
    knob = object_model.get_part("dispense_knob")
    flap = object_model.get_part("chute_flap")
    lid_hinge = object_model.get_articulation("body_to_refill_lid")
    knob_joint = object_model.get_articulation("body_to_dispense_knob")
    flap_hinge = object_model.get_articulation("body_to_chute_flap")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_cap",
        elem_b="top_opening_rim",
        min_overlap=0.12,
        name="refill lid covers the open crown rim",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_cap",
        negative_elem="top_opening_rim",
        min_gap=-0.004,
        max_gap=0.030,
        name="refill lid sits on the top rim",
    )
    ctx.expect_contact(
        knob,
        body,
        elem_a="shaft_collar",
        elem_b="knob_bushing",
        contact_tol=0.004,
        name="dispense knob collar is seated on bushing",
    )
    ctx.check(
        "dispense knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_cap")
    with ctx.pose({lid_hinge: 1.55}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_cap")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_cap",
            negative_elem="top_opening_rim",
            min_gap=0.020,
            name="refill lid lifts clear of crown opening",
        )
    ctx.check(
        "refill lid hinge opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.030,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_hinge: 0.85}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "chute flap opens outward and down",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][0] > closed_flap_aabb[1][0] + 0.025
        and open_flap_aabb[0][2] < closed_flap_aabb[0][2] - 0.010,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    with ctx.pose({knob_joint: math.pi / 2.0}):
        ctx.expect_contact(
            knob,
            body,
            elem_a="shaft_collar",
            elem_b="knob_bushing",
            contact_tol=0.004,
            name="rotating knob remains seated on shaft axis",
        )

    return ctx.report()


object_model = build_object_model()
