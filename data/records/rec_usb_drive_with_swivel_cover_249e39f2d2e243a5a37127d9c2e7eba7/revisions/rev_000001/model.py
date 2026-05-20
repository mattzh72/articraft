from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


PIVOT_X = 0.032
PIVOT_Y = 0.026


def _rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    """A small filleted rectangular solid, authored in meters."""
    return cq.Workplane("XY").box(length, width, height).edges().fillet(radius)


def _rect_frame_yz(
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    thickness_x: float,
) -> cq.Workplane:
    """A rectangular gasket/frame in the YZ plane, extruded along +X."""
    return (
        cq.Workplane("YZ")
        .rect(outer_y, outer_z)
        .rect(inner_y, inner_z)
        .extrude(thickness_x)
    )


def _cover_shell() -> cq.Workplane:
    """Hollow swivel cap with an integral pivot bushing and side bridge."""
    # Local frame origin is the pivot axis.  At q=0 the cap lies along +X over
    # the USB connector while its bushing surrounds the visible body pin.
    shell_min_x = 0.013
    shell_length = 0.047
    shell_width = 0.034
    shell_height = 0.018
    shell_center_x = shell_min_x + shell_length / 2.0
    shell_center_y = -PIVOT_Y

    outer = (
        cq.Workplane("XY")
        .box(shell_length, shell_width, shell_height)
        .translate((shell_center_x, shell_center_y, 0.0))
        .edges()
        .fillet(0.0025)
    )

    # Cut a rear-opening connector cavity; the front wall remains closed.
    cavity = (
        cq.Workplane("XY")
        .box(0.046, 0.024, 0.011)
        .translate((shell_min_x + 0.019, shell_center_y, 0.0))
    )
    shell = outer.cut(cavity)

    # Annular bushing around the pivot pin.  The pin itself belongs to the body;
    # this clearanced hole keeps the moving cover from colliding with it.
    lug_height = 0.009
    lug = (
        cq.Workplane("XY")
        .circle(0.0058)
        .circle(0.0031)
        .extrude(lug_height)
        .translate((0.0, 0.0, -lug_height / 2.0))
    )

    # A thick side bridge ties the bushing hard into the cap wall.
    bridge = (
        cq.Workplane("XY")
        .box(0.037, 0.006, 0.005)
        .translate((0.0185, -0.0065, 0.0))
        .edges("|X")
        .fillet(0.0012)
    )

    return shell.union(lug).union(bridge)


def _cover_rear_gasket() -> cq.Workplane:
    """Soft rectangular rear seal carried by the swivel cover."""
    gasket = _rect_frame_yz(0.031, 0.015, 0.024, 0.010, 0.0024)
    # Slightly straddles the rear edge of the cap, so it reads as seated.
    return gasket.translate((0.0128, -PIVOT_Y, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_usb_drive")

    rubber = model.material("matte_black_overmold", rgba=(0.015, 0.017, 0.018, 1.0))
    dark = model.material("dark_polycarbonate_cover", rgba=(0.035, 0.040, 0.045, 1.0))
    seal = model.material("blue_silicone_seals", rgba=(0.02, 0.30, 0.78, 1.0))
    stainless = model.material("brushed_stainless_hardware", rgba=(0.70, 0.72, 0.70, 1.0))
    metal = model.material("nickel_plated_usb_shell", rgba=(0.78, 0.77, 0.72, 1.0))
    plastic = model.material("black_connector_insert", rgba=(0.003, 0.003, 0.004, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_box(0.088, 0.032, 0.018, 0.004), "body_overmold"),
        material=rubber,
        name="body_overmold",
    )

    # Raised lips and side rails make the housing look rubber overmolded and
    # keep water from tracking straight toward the connector seam.
    for y, name in ((0.0125, "top_rail_0"), (-0.0125, "top_rail_1")):
        body.visual(
            Box((0.074, 0.0042, 0.0024)),
            origin=Origin(xyz=(-0.002, y, 0.0092)),
            material=rubber,
            name=name,
        )

    body.visual(
        Box((0.010, 0.030, 0.0022)),
        origin=Origin(xyz=(0.0405, 0.0, 0.0101)),
        material=rubber,
        name="drip_brow",
    )
    body.visual(
        Box((0.010, 0.030, 0.0020)),
        origin=Origin(xyz=(0.0405, 0.0, -0.0100)),
        material=rubber,
        name="lower_drip_lip",
    )

    body.visual(
        mesh_from_cadquery(
            _rect_frame_yz(0.027, 0.014, 0.015, 0.0065, 0.0022).translate((0.0424, 0.0, 0.0)),
            "front_seal",
        ),
        material=seal,
        name="front_seal",
    )

    # USB-A connector, intentionally rooted into the body overmold.
    body.visual(
        Box((0.019, 0.012, 0.0045)),
        origin=Origin(xyz=(0.0530, 0.0, 0.0)),
        material=metal,
        name="usb_shell",
    )
    body.visual(
        Box((0.0075, 0.0072, 0.0015)),
        origin=Origin(xyz=(0.0618, 0.0, 0.0)),
        material=plastic,
        name="usb_tongue",
    )
    for y, name in ((0.0033, "usb_retention_0"), (-0.0033, "usb_retention_1")):
        body.visual(
            Box((0.0035, 0.0021, 0.0008)),
            origin=Origin(xyz=(0.0548, y, 0.00235)),
            material=plastic,
            name=name,
        )

    # A two-ear clevis captures the cover bushing; the moving bushing occupies
    # the middle gap while the stainless pin passes through all layers.
    for z, name in ((0.0075, "pivot_ear_top"), (-0.0075, "pivot_ear_bottom")):
        body.visual(
            Box((0.015, 0.011, 0.004)),
            origin=Origin(xyz=(PIVOT_X, 0.0212, z)),
            material=rubber,
            name=name,
        )

    body.visual(
        Cylinder(radius=0.0024, length=0.024),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0)),
        material=stainless,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.0016),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0122)),
        material=stainless,
        name="pin_head_top",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.0016),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, -0.0122)),
        material=stainless,
        name="pin_head_bottom",
    )

    for x, name in ((-0.024, "case_screw_0"), (0.010, "case_screw_1")):
        body.visual(
            Cylinder(radius=0.0026, length=0.0012),
            origin=Origin(xyz=(x, -0.0105, 0.0094)),
            material=stainless,
            name=name,
        )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_shell(), "cover_shell"),
        material=dark,
        name="cover_shell",
    )
    cover.visual(
        mesh_from_cadquery(_cover_rear_gasket(), "cover_gasket"),
        material=seal,
        name="cover_gasket",
    )
    cover.visual(
        Box((0.017, 0.038, 0.0022)),
        origin=Origin(xyz=(0.018, -PIVOT_Y, 0.0101)),
        material=dark,
        name="cover_drip_overhang",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=0.0, upper=2.05),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    hinge = object_model.get_articulation("body_to_cover")

    # At the sealed pose the cover's gasket should land just in front of the
    # body gasket without penetrating it.
    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            positive_elem="cover_gasket",
            negative_elem="front_seal",
            min_gap=0.0,
            max_gap=0.0015,
            name="closed cover gasket seats at body seal",
        )
        ctx.expect_within(
            body,
            cover,
            axes="yz",
            inner_elem="usb_shell",
            outer_elem="cover_shell",
            margin=0.003,
            name="closed cap surrounds USB plug clearance",
        )
        ctx.expect_overlap(
            body,
            cover,
            axes="x",
            elem_a="usb_shell",
            elem_b="cover_shell",
            min_overlap=0.010,
            name="closed cap covers connector length",
        )
        ctx.expect_within(
            body,
            cover,
            axes="xy",
            inner_elem="pivot_pin",
            outer_elem="cover_shell",
            margin=0.001,
            name="visible pin remains inside cover bushing footprint",
        )

        closed_aabb = ctx.part_element_world_aabb(cover, elem="cover_shell")

    with ctx.pose({hinge: 1.55}):
        open_aabb = ctx.part_element_world_aabb(cover, elem="cover_shell")
        ctx.expect_gap(
            cover,
            body,
            axis="y",
            positive_elem="cover_shell",
            negative_elem="body_overmold",
            min_gap=-0.001,
            name="opened cover swings to side of body",
        )

    ctx.check(
        "cover swings outward around side pin",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.020,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
