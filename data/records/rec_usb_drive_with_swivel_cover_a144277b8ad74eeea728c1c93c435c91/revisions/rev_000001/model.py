from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_body_mesh():
    return mesh_from_cadquery(
        cq.Workplane("XY")
        .box(0.060, 0.020, 0.009)
        .edges()
        .fillet(0.0016),
        "rounded_drive_body",
        tolerance=0.00035,
        angular_tolerance=0.08,
    )


def _washer_mesh(name: str, y_center: float):
    thickness = 0.0012
    return mesh_from_cadquery(
        cq.Workplane("XZ")
        .circle(0.0054)
        .circle(0.0024)
        .extrude(thickness)
        .translate((0.0, y_center + thickness / 2.0, 0.0)),
        name,
        tolerance=0.0002,
        angular_tolerance=0.08,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="legacy_swivel_usb_drive")

    bakelite = model.material("aged_black_bakelite", rgba=(0.025, 0.027, 0.024, 1.0))
    dark_panel = model.material("service_panel_black", rgba=(0.010, 0.012, 0.012, 1.0))
    gunmetal = model.material("gunmetal_adapter", rgba=(0.30, 0.31, 0.29, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    bright_steel = model.material("bright_pin_heads", rgba=(0.86, 0.86, 0.82, 1.0))
    brass = model.material("brass_fasteners", rgba=(0.88, 0.63, 0.22, 1.0))
    usb_blue = model.material("usb_blue_tongue", rgba=(0.05, 0.20, 0.52, 1.0))
    gold = model.material("gold_contacts", rgba=(1.00, 0.78, 0.18, 1.0))

    core = model.part("core")
    cover = model.part("swivel_cover")

    # Main memory-stick body: rounded old bakelite-like shell, 60 mm long.
    core.visual(
        _rounded_body_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=bakelite,
        name="body_shell",
    )

    # Raised service hatches and stamped ribs deliberately read as retrofit,
    # field-serviceable construction rather than a seamless modern capsule.
    core.visual(
        Box((0.026, 0.012, 0.0007)),
        origin=Origin(xyz=(-0.004, 0.000, 0.00955)),
        material=dark_panel,
        name="top_service_hatch",
    )
    core.visual(
        Box((0.012, 0.014, 0.0006)),
        origin=Origin(xyz=(-0.023, 0.000, 0.00948)),
        material=gunmetal,
        name="rear_service_hatch",
    )
    for i, x in enumerate((-0.015, 0.007)):
        for j, y in enumerate((-0.0044, 0.0044)):
            core.visual(
                Cylinder(radius=0.00075, length=0.00055),
                origin=Origin(xyz=(x, y, 0.01005)),
                material=brass,
                name=f"hatch_screw_{i}_{j}",
            )

    # Side reinforcement straps tie the rear pivot boss into the body shell.
    for side, y in enumerate((-0.01075, 0.01075)):
        core.visual(
            Box((0.020, 0.0012, 0.0030)),
            origin=Origin(xyz=(-0.020, y, 0.0063)),
            material=gunmetal,
            name=f"side_reinforcement_{side}",
        )
        core.visual(
            Cylinder(radius=0.0048, length=0.0015),
            origin=Origin(xyz=(-0.026, y, 0.0095), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"pivot_boss_{side}",
        )

    # Visible through-pin and peened heads: this is the actual swivel constraint.
    core.visual(
        Cylinder(radius=0.00235, length=0.0282),
        origin=Origin(xyz=(-0.026, 0.0, 0.0095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="pivot_pin",
    )
    core.visual(
        Cylinder(radius=0.00365, length=0.0013),
        origin=Origin(xyz=(-0.026, 0.01335, 0.0095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="pivot_pin_head_0",
    )
    core.visual(
        Cylinder(radius=0.00365, length=0.0013),
        origin=Origin(xyz=(-0.026, -0.01335, 0.0095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="pivot_pin_head_1",
    )

    # Bolted front adapter collar supporting the USB-A connector.
    core.visual(
        Box((0.0020, 0.0190, 0.0014)),
        origin=Origin(xyz=(0.0310, 0.0, 0.0085)),
        material=gunmetal,
        name="adapter_top_rail",
    )
    core.visual(
        Box((0.0020, 0.0190, 0.0012)),
        origin=Origin(xyz=(0.0310, 0.0, 0.0011)),
        material=gunmetal,
        name="adapter_bottom_rail",
    )
    for side, y in enumerate((-0.0087, 0.0087)):
        core.visual(
            Box((0.0020, 0.0014, 0.0072)),
            origin=Origin(xyz=(0.0310, y, 0.0048)),
            material=gunmetal,
            name=f"adapter_side_rail_{side}",
        )
    for i, y in enumerate((-0.0070, 0.0070)):
        for j, z in enumerate((0.0024, 0.0076)):
            core.visual(
                Cylinder(radius=0.00075, length=0.00065),
                origin=Origin(xyz=(0.0322, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=brass,
                name=f"adapter_bolt_{i}_{j}",
            )

    # USB-A connector shell is modeled as thin plates with an open mouth, not a
    # solid block; the blue tongue and contacts are visibly recessed inside.
    core.visual(
        Box((0.0180, 0.0126, 0.00075)),
        origin=Origin(xyz=(0.0410, 0.0, 0.00705)),
        material=brushed_steel,
        name="usb_shell_top",
    )
    core.visual(
        Box((0.0180, 0.0126, 0.00065)),
        origin=Origin(xyz=(0.0410, 0.0, 0.00215)),
        material=brushed_steel,
        name="usb_shell_bottom",
    )
    for side, y in enumerate((-0.0060, 0.0060)):
        core.visual(
            Box((0.0180, 0.00075, 0.0049)),
            origin=Origin(xyz=(0.0410, y, 0.0046)),
            material=brushed_steel,
            name=f"usb_shell_side_{side}",
        )
    core.visual(
        Box((0.0130, 0.0068, 0.0010)),
        origin=Origin(xyz=(0.0430, 0.0, 0.00455)),
        material=usb_blue,
        name="usb_tongue",
    )
    core.visual(
        Box((0.0032, 0.0072, 0.0036)),
        origin=Origin(xyz=(0.0352, 0.0, 0.00355)),
        material=usb_blue,
        name="usb_tongue_root",
    )
    for i, y in enumerate((-0.0027, -0.0009, 0.0009, 0.0027)):
        core.visual(
            Box((0.0035, 0.0011, 0.00025)),
            origin=Origin(xyz=(0.0472, y, 0.00514)),
            material=gold,
            name=f"usb_contact_{i}",
        )

    # The swivel cover is a thin U-channel.  Its local frame is the pivot axis;
    # in the closed pose it extends in local +X over the connector.
    cover.visual(
        Box((0.078, 0.0240, 0.0016)),
        origin=Origin(xyz=(0.042, 0.0, 0.0040)),
        material=brushed_steel,
        name="cover_top",
    )
    for side, y in enumerate((-0.0121, 0.0121)):
        cover.visual(
            Box((0.076, 0.0012, 0.0090)),
            origin=Origin(xyz=(0.043, y, -0.00055)),
            material=brushed_steel,
            name=f"cover_side_{side}",
        )
    cover.visual(
        Box((0.0014, 0.0240, 0.0078)),
        origin=Origin(xyz=(0.081, 0.0, -0.0001)),
        material=brushed_steel,
        name="cover_nose_lip",
    )
    for i, y in enumerate((-0.0060, 0.0060)):
        cover.visual(
            Box((0.053, 0.0011, 0.00075)),
            origin=Origin(xyz=(0.045, y, 0.00515)),
            material=bright_steel,
            name=f"cover_stiffener_{i}",
        )
    cover.visual(
        _washer_mesh("cover_washer_0", -0.0121),
        material=brushed_steel,
        name="pivot_washer_0",
    )
    cover.visual(
        _washer_mesh("cover_washer_1", 0.0121),
        material=brushed_steel,
        name="pivot_washer_1",
    )

    model.articulation(
        "core_to_cover",
        ArticulationType.REVOLUTE,
        parent=core,
        child=cover,
        origin=Origin(xyz=(-0.026, 0.0, 0.0095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(170.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    core = object_model.get_part("core")
    cover = object_model.get_part("swivel_cover")
    swivel = object_model.get_articulation("core_to_cover")

    with ctx.pose({swivel: 0.0}):
        ctx.expect_gap(
            cover,
            core,
            axis="z",
            positive_elem="cover_top",
            negative_elem="body_shell",
            min_gap=0.002,
            name="closed cover clears the body top",
        )
        ctx.expect_overlap(
            cover,
            core,
            axes="x",
            elem_a="cover_top",
            elem_b="usb_shell_top",
            min_overlap=0.012,
            name="closed cover spans the USB connector",
        )
        ctx.expect_contact(
            core,
            cover,
            elem_a="pivot_pin_head_0",
            elem_b="pivot_washer_1",
            contact_tol=0.00025,
            name="visible pin head retains one cover washer",
        )
        ctx.expect_contact(
            core,
            cover,
            elem_a="pivot_pin_head_1",
            elem_b="pivot_washer_0",
            contact_tol=0.00025,
            name="visible pin head retains opposite cover washer",
        )

    rest_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({swivel: math.radians(150.0)}):
        open_aabb = ctx.part_world_aabb(cover)
        ctx.expect_gap(
            cover,
            core,
            axis="z",
            positive_elem="cover_nose_lip",
            negative_elem="usb_shell_top",
            min_gap=0.006,
            name="swiveled cover lifts away from connector",
        )
    ctx.check(
        "cover swings about the visible side pin",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.035,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
