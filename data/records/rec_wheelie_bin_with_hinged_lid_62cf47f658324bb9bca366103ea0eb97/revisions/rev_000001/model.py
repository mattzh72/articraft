from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BIN_TOP_Z = 1.02
BIN_BOTTOM_Z = 0.16
HINGE_X = -0.410
HINGE_Z = 1.065
WHEEL_X = -0.315
WHEEL_Z = 0.155
WHEEL_Y = 0.365
WHEEL_RADIUS = 0.130
WHEEL_WIDTH = 0.066


def _tapered_hollow_body() -> cq.Workplane:
    """Open tapered wheelie-bin tub: thick bottom, sloped side walls, open top."""
    height = BIN_TOP_Z - BIN_BOTTOM_Z
    bottom_depth, bottom_width = 0.44, 0.43
    top_depth, top_width = 0.68, 0.58
    wall = 0.035
    bottom_thickness = 0.055

    outer = (
        cq.Workplane("XY")
        .rect(bottom_depth, bottom_width)
        .workplane(offset=height)
        .rect(top_depth, top_width)
        .loft(ruled=True)
    )

    inner = (
        cq.Workplane("XY")
        .workplane(offset=bottom_thickness)
        .rect(bottom_depth - 2.0 * wall, bottom_width - 2.0 * wall)
        .workplane(offset=height + 0.08 - bottom_thickness)
        .rect(top_depth - 2.0 * wall, top_width - 2.0 * wall)
        .loft(ruled=True)
    )
    return outer.cut(inner).translate((0.0, 0.0, BIN_BOTTOM_Z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    bin_green = Material("molded_green_plastic", color=(0.06, 0.32, 0.16, 1.0))
    darker_green = Material("darker_lid_plastic", color=(0.04, 0.24, 0.12, 1.0))
    black_rubber = Material("black_rubber", color=(0.01, 0.01, 0.01, 1.0))
    dark_metal = Material("blackened_steel", color=(0.03, 0.03, 0.035, 1.0))
    grey_plastic = Material("grey_wheel_hub", color=(0.42, 0.43, 0.40, 1.0))

    body = model.part("bin_body")
    body.visual(
        mesh_from_cadquery(_tapered_hollow_body(), "open_tapered_bin_body", tolerance=0.002),
        material=bin_green,
        name="body_shell",
    )

    # Rolled top rim, modeled as overlapping plastic bars so the open tub reads
    # as a hollow molded bin rather than a capped solid.
    body.visual(
        Box((0.058, 0.660, 0.055)),
        origin=Origin(xyz=(-0.356, 0.0, 1.040)),
        material=bin_green,
        name="rear_rim",
    )
    body.visual(
        Box((0.058, 0.660, 0.055)),
        origin=Origin(xyz=(0.356, 0.0, 1.040)),
        material=bin_green,
        name="front_rim",
    )
    for y, name in ((-0.306, "rim_side_0"), (0.306, "rim_side_1")):
        body.visual(
            Box((0.730, 0.058, 0.055)),
            origin=Origin(xyz=(0.0, y, 1.040)),
            material=bin_green,
            name=name,
        )

    # Rear axle and molded supports tucked under the back of the bin.
    body.visual(
        Cylinder(radius=0.018, length=0.800),
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle",
    )
    for y, name in ((-0.215, "axle_mount_0"), (0.215, "axle_mount_1")):
        body.visual(
            Box((0.115, 0.080, 0.100)),
            origin=Origin(xyz=(-0.265, y, 0.190)),
            material=bin_green,
            name=name,
        )

    # Compact hinge hardware: short barrels and small brackets close to the rear rim.
    for y, name in ((-0.235, "hinge_barrel_0"), (0.235, "hinge_barrel_1")):
        body.visual(
            Cylinder(radius=0.022, length=0.075),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=name,
        )
        body.visual(
            Box((0.040, 0.082, 0.055)),
            origin=Origin(xyz=(-0.402, y, 1.035)),
            material=bin_green,
            name=f"hinge_bracket_{name[-1]}",
        )

    # A rear grab handle molded into the body below the hinge line.
    body.visual(
        Cylinder(radius=0.020, length=0.590),
        origin=Origin(xyz=(-0.388, 0.0, 0.965), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bin_green,
        name="rear_handle",
    )
    for y, name in ((-0.290, "handle_post_0"), (0.290, "handle_post_1")):
        body.visual(
            Box((0.100, 0.050, 0.130)),
            origin=Origin(xyz=(-0.370, y, 0.925)),
            material=bin_green,
            name=name,
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.800, 0.720, 0.035)),
        origin=Origin(xyz=(0.430, 0.0, 0.020)),
        material=darker_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.610, 0.545, 0.014)),
        origin=Origin(xyz=(0.425, 0.0, 0.043)),
        material=bin_green,
        name="raised_lid_panel",
    )
    lid.visual(
        Box((0.036, 0.640, 0.060)),
        origin=Origin(xyz=(0.842, 0.0, 0.000)),
        material=darker_green,
        name="front_lip",
    )
    for y, name in ((-0.120, "lid_knuckle_0"), (0.0, "lid_knuckle_1"), (0.120, "lid_knuckle_2")):
        lid.visual(
            Cylinder(radius=0.018, length=0.055),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=name,
        )
        lid.visual(
            Box((0.074, 0.055, 0.030)),
            origin=Origin(xyz=(0.034, y, 0.018)),
            material=darker_green,
            name=f"lid_hinge_tab_{name[-1]}",
        )

    tire = TireGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        inner_radius=0.088,
        tread=TireTread(style="block", depth=0.008, count=18, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.007, depth=0.003),),
        sidewall=TireSidewall(style="square", bulge=0.02),
        shoulder=TireShoulder(width=0.007, radius=0.003),
    )
    wheel_face = WheelGeometry(
        0.091,
        WHEEL_WIDTH * 0.86,
        rim=WheelRim(inner_radius=0.060, flange_height=0.006, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.030,
            width=0.044,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.040, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.003, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.006, window_radius=0.012),
        bore=WheelBore(style="round", diameter=0.028),
    )

    for y, name in ((-WHEEL_Y, "wheel_0"), (WHEEL_Y, "wheel_1")):
        wheel = model.part(name)
        wheel.visual(
            mesh_from_geometry(tire, f"{name}_tire"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black_rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(wheel_face, f"{name}_hub"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=grey_plastic,
            name="hub",
        )
        model.articulation(
            f"bin_body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(WHEEL_X, y, WHEEL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    model.articulation(
        "bin_body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bin_body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("bin_body_to_lid")

    ctx.check(
        "lid hinge is rear revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in lid_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}",
    )
    for wheel_name in ("wheel_0", "wheel_1"):
        wheel = object_model.get_part(wheel_name)
        joint = object_model.get_articulation(f"bin_body_to_{wheel_name}")
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="rear_axle",
            elem_b="hub",
            reason="The steel rear axle is intentionally captured through the simplified plastic hub bore.",
        )
        ctx.expect_within(
            body,
            wheel,
            axes="xz",
            inner_elem="rear_axle",
            outer_elem="hub",
            margin=0.0,
            name=f"{wheel_name} hub surrounds axle on radial axes",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            elem_a="rear_axle",
            elem_b="hub",
            min_overlap=0.045,
            name=f"{wheel_name} remains on rear axle",
        )
        ctx.check(
            f"{wheel_name} spins about rear axle",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 3) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.003,
            max_penetration=0.0005,
            positive_elem="lid_panel",
            negative_elem="front_rim",
            name="closed lid rests on front rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="body_shell",
            min_overlap=0.50,
            name="full width lid covers bin opening",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    def zmax(aabb):
        if aabb is None:
            return None
        upper = aabb[1]
        return upper[2] if hasattr(upper, "__getitem__") else upper.z

    ctx.check(
        "lid opens upward from rear hinge",
        zmax(closed_lid_aabb) is not None
        and zmax(open_lid_aabb) is not None
        and zmax(open_lid_aabb) > zmax(closed_lid_aabb) + 0.25,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
