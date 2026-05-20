from __future__ import annotations

import math

import cadquery as cq

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


def _cq_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> cq.Workplane:
    """CadQuery box helper using meter units and an explicit center."""
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _build_body_shell() -> cq.Workplane:
    """One-piece hollow weatherproof storage tub with molded anchor lugs."""
    width = 0.420
    depth = 0.280
    height = 0.200
    wall = 0.018
    floor = 0.022

    outer = _cq_box((width, depth, height), (0.0, 0.0, height / 2.0))
    inner_cut = _cq_box(
        (width - 2.0 * wall, depth - 2.0 * wall, height),
        (0.0, 0.0, floor + height / 2.0),
    )
    shell = outer.cut(inner_cut)

    # Four integral feet/anchor tabs are molded into the lower tub so the box
    # reads as a practical outdoor container that can be screwed to a shelf.
    lug_w = 0.080
    lug_d = 0.050
    lug_h = 0.016
    lug_overlap = 0.004
    for x in (-0.130, 0.130):
        for y_sign in (-1.0, 1.0):
            lug_y = y_sign * (depth / 2.0 + lug_d / 2.0 - lug_overlap)
            shell = shell.union(_cq_box((lug_w, lug_d, lug_h), (x, lug_y, lug_h / 2.0)))

    return shell


def _build_lid_shell() -> cq.Workplane:
    """One-piece rain-shedding lid with drip skirts and a rear hinge hood."""
    lid_w = 0.490
    lid_d = 0.335
    lid_t = 0.026
    hinge_to_rear_edge = 0.010
    hinge_to_front_edge = hinge_to_rear_edge + lid_d
    lid_y_center = -(hinge_to_front_edge + hinge_to_rear_edge) / 2.0

    # Hinge frame is at the barrel centerline.  The lid's seal surface is above
    # that axis, just like a real utility box hinge mounted below the rim.
    top_bottom_z = 0.018
    top_center_z = top_bottom_z + lid_t / 2.0
    lid = _cq_box((lid_w, lid_d, lid_t), (0.0, lid_y_center, top_center_z))

    skirt_t = 0.012
    skirt_h = 0.040
    skirt_center_z = top_bottom_z - skirt_h / 2.0
    front_y = -hinge_to_front_edge
    rear_y = -hinge_to_rear_edge

    lid = lid.union(
        _cq_box(
            (lid_w, skirt_t, skirt_h),
            (0.0, front_y + skirt_t / 2.0, skirt_center_z),
        )
    )
    for x_sign in (-1.0, 1.0):
        lid = lid.union(
            _cq_box(
                (skirt_t, lid_d, skirt_h),
                (x_sign * (lid_w / 2.0 - skirt_t / 2.0), lid_y_center, skirt_center_z),
            )
        )

    # Raised rear hood shields the hinge barrels from direct rain while staying
    # attached to the lid skin for a fabricable silhouette.
    hood = _cq_box((lid_w, 0.032, 0.010), (0.0, rear_y + 0.016, top_center_z + lid_t / 2.0 + 0.001))
    lid = lid.union(hood)

    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_sewing_box")

    body_plastic = model.material("olive_weatherproof_plastic", rgba=(0.12, 0.24, 0.16, 1.0))
    lid_plastic = model.material("sage_uv_stable_lid", rgba=(0.20, 0.36, 0.24, 1.0))
    gasket = model.material("black_epdm_gasket", rgba=(0.015, 0.014, 0.012, 1.0))
    stainless = model.material("brushed_stainless_hardware", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_label = model.material("dark_molded_label", rgba=(0.045, 0.055, 0.050, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "body_shell", tolerance=0.001),
        material=body_plastic,
        name="body_shell",
    )

    # Raised EPDM ring on the rim: four hard-mounted strips, visible as the
    # sealed interface that the lid's underside land closes over.
    body.visual(
        Box((0.360, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.131, 0.203)),
        material=gasket,
        name="front_gasket",
    )
    body.visual(
        Box((0.360, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.131, 0.203)),
        material=gasket,
        name="rear_gasket",
    )
    for x, name in ((-0.201, "side_gasket_0"), (0.201, "side_gasket_1")):
        body.visual(
            Box((0.012, 0.232, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.203)),
            material=gasket,
            name=name,
        )

    hinge_y = 0.170
    hinge_z = 0.190
    knuckle_radius = 0.008
    body_knuckle_segments = (
        (-0.150, 0.110, "body_knuckle_0"),
        (0.150, 0.110, "body_knuckle_1"),
    )
    for x, length, name in body_knuckle_segments:
        body.visual(
            Cylinder(radius=knuckle_radius, length=length),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name=name,
        )
        # Body-side hinge leaf and strap make the barrel visibly continuous with
        # the rear wall rather than floating behind the box.
        body.visual(
            Box((length, 0.004, 0.060)),
            origin=Origin(xyz=(x, 0.142, 0.170)),
            material=stainless,
            name=f"hinge_leaf_{name[-1]}",
        )
        body.visual(
            Box((length, 0.030, 0.006)),
            origin=Origin(xyz=(x, 0.156, hinge_z)),
            material=stainless,
            name=f"hinge_strap_{name[-1]}",
        )

    # Continuous stainless pin through all hinge barrels.  It is authored on the
    # grounded body side and captured by the lid barrel to give the moving lid a
    # real mechanical support path.
    body.visual(
        Cylinder(radius=0.0035, length=0.420),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_pin",
    )

    # Stainless anchor screw heads on the integral lugs.
    for i, (x, y) in enumerate(
        (
            (-0.130, -0.161),
            (0.130, -0.161),
            (-0.130, 0.161),
            (0.130, 0.161),
        )
    ):
        body.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x, y, 0.018)),
            material=stainless,
            name=f"anchor_screw_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "lid_shell", tolerance=0.001),
        material=lid_plastic,
        name="lid_shell",
    )

    # Underside seal lands are thin molded pads, lightly tucked into the lid so
    # they remain mechanically continuous while giving exact gasket references.
    lid.visual(
        Box((0.360, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, -0.301, 0.0178)),
        material=lid_plastic,
        name="front_seal_land",
    )
    lid.visual(
        Box((0.360, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, -0.039, 0.0178)),
        material=lid_plastic,
        name="rear_seal_land",
    )
    for x, name in ((-0.201, "side_seal_land_0"), (0.201, "side_seal_land_1")):
        lid.visual(
            Box((0.012, 0.232, 0.002)),
            origin=Origin(xyz=(x, -0.170, 0.0178)),
            material=lid_plastic,
            name=name,
        )

    # Lid-side hinge hardware: an alternating center knuckle plus a formed leaf
    # that reaches upward to the underside of the lid.
    lid.visual(
        Cylinder(radius=knuckle_radius, length=0.176),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="lid_knuckle",
    )
    lid.visual(
        Box((0.176, 0.006, 0.034)),
        origin=Origin(xyz=(0.0, -0.010, 0.006)),
        material=stainless,
        name="lid_hinge_leaf",
    )

    # Molded top label with a simple raised spool-and-needle icon so the object
    # still reads as a sewing box despite the outdoor utility construction.
    lid.visual(
        Box((0.150, 0.060, 0.004)),
        origin=Origin(xyz=(0.0, -0.185, 0.046)),
        material=dark_label,
        name="sewing_label",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.005),
        origin=Origin(xyz=(-0.025, -0.185, 0.0505)),
        material=stainless,
        name="spool_cap_0",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.005),
        origin=Origin(xyz=(0.025, -0.185, 0.0505)),
        material=stainless,
        name="spool_cap_1",
    )
    lid.visual(
        Box((0.050, 0.010, 0.005)),
        origin=Origin(xyz=(0.0, -0.185, 0.0505)),
        material=stainless,
        name="spool_core",
    )
    lid.visual(
        Box((0.006, 0.070, 0.004)),
        origin=Origin(xyz=(0.045, -0.185, 0.049), rpy=(0.0, 0.0, -0.55)),
        material=stainless,
        name="needle_mark",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        # The closed lid extends toward -Y from the rear hinge, so -X makes
        # positive rotation lift the front edge upward and back.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.65),
        motion_properties=MotionProperties(damping=0.08, friction=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_pin",
        elem_b="lid_knuckle",
        reason=(
            "The stainless hinge pin is intentionally captured inside the "
            "lid barrel; the simplified barrel is solid rather than hollow."
        ),
    )

    ctx.check(
        "rear lid hinge is bounded",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and 1.4 <= hinge.motion_limits.upper <= 1.8,
        details=f"limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_seal_land",
            negative_elem="front_gasket",
            min_gap=0.0,
            max_gap=0.0015,
            name="front seal closes over gasket",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="front_seal_land",
            elem_b="front_gasket",
            min_overlap=0.010,
            name="front seal land covers gasket",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="rear_seal_land",
            negative_elem="rear_gasket",
            min_gap=0.0,
            max_gap=0.0015,
            name="rear seal closes over gasket",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            positive_elem="lid_knuckle",
            negative_elem="body_knuckle_0",
            min_gap=0.004,
            max_gap=0.018,
            name="left hinge knuckle clearance",
        )
        ctx.expect_gap(
            body,
            lid,
            axis="x",
            positive_elem="body_knuckle_1",
            negative_elem="lid_knuckle",
            min_gap=0.004,
            max_gap=0.018,
            name="right hinge knuckle clearance",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            elem_a="lid_knuckle",
            elem_b="body_knuckle_0",
            min_overlap=0.010,
            name="hinge barrels share pin axis",
        )
        ctx.expect_within(
            body,
            lid,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem="lid_knuckle",
            margin=0.0,
            name="hinge pin is centered in lid barrel",
        )
        ctx.expect_overlap(
            body,
            lid,
            axes="x",
            elem_a="hinge_pin",
            elem_b="lid_knuckle",
            min_overlap=0.160,
            name="hinge pin spans lid barrel",
        )

        lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        front_gasket_aabb = ctx.part_element_world_aabb(body, elem="front_gasket")
        side_gasket_0_aabb = ctx.part_element_world_aabb(body, elem="side_gasket_0")
        side_gasket_1_aabb = ctx.part_element_world_aabb(body, elem="side_gasket_1")
        ctx.check(
            "lid has drip overhang over seal",
            lid_aabb is not None
            and front_gasket_aabb is not None
            and side_gasket_0_aabb is not None
            and side_gasket_1_aabb is not None
            and lid_aabb[0][0] < side_gasket_0_aabb[0][0] - 0.025
            and lid_aabb[1][0] > side_gasket_1_aabb[1][0] + 0.025
            and lid_aabb[0][1] < front_gasket_aabb[0][1] - 0.020,
            details=(
                f"lid_aabb={lid_aabb}, front_gasket={front_gasket_aabb}, "
                f"side_gaskets={(side_gasket_0_aabb, side_gasket_1_aabb)}"
            ),
        )

        closed_front = ctx.part_element_world_aabb(lid, elem="front_seal_land")

    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_seal_land")
        ctx.check(
            "hinged lid lifts front edge",
            closed_front is not None
            and open_front is not None
            and open_front[0][2] > closed_front[0][2] + 0.18,
            details=f"closed={closed_front}, open={open_front}",
        )

    return ctx.report()


object_model = build_object_model()
