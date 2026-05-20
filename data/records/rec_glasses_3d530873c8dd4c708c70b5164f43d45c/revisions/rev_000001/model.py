from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _sleeve_mesh() -> cq.Workplane:
    """Rectangular hollow temple sleeve, open through its length."""
    outer_x = 0.010
    length_y = 0.082
    outer_z = 0.010
    inner_x = 0.0060
    inner_z = 0.0062
    outer = cq.Workplane("XY").box(outer_x, length_y, outer_z)
    bore = cq.Workplane("XY").box(inner_x, length_y + 0.010, inner_z)
    return outer.cut(bore)


def _side_shield_mesh() -> cq.Workplane:
    """Thin trapezoid shield plate in the local YZ plane, extruded in X."""
    length_y = 0.057
    height_z = 0.039
    thickness_x = 0.0026
    pts = [
        (0.000, -height_z * 0.47),
        (length_y * 0.84, -height_z * 0.39),
        (length_y, height_z * 0.24),
        (length_y * 0.35, height_z * 0.50),
        (0.000, height_z * 0.43),
    ]
    return cq.Workplane("YZ").polyline(pts).close().extrude(thickness_x, both=True)


def _lens_plate_mesh(width: float, height: float, thickness_y: float) -> cq.Workplane:
    """Octagonal thin lens plate in the XZ plane."""
    w = width / 2.0
    h = height / 2.0
    pts = [
        (-w * 0.78, -h),
        (w * 0.78, -h),
        (w, -h * 0.56),
        (w * 0.94, h * 0.56),
        (w * 0.62, h),
        (-w * 0.62, h),
        (-w * 0.94, h * 0.56),
        (-w, -h * 0.56),
    ]
    return cq.Workplane("XZ").polyline(pts).close().extrude(thickness_y, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_safety_glasses")

    clear_lens = model.material("clear_polycarbonate", rgba=(0.52, 0.85, 1.0, 0.36))
    black_frame = model.material("matte_black_frame", rgba=(0.02, 0.025, 0.03, 1.0))
    temple_plastic = model.material("dark_gray_temple", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber = model.material("soft_rubber_tips", rgba=(0.01, 0.012, 0.014, 1.0))
    metal = model.material("brushed_pin_metal", rgba=(0.55, 0.55, 0.52, 1.0))

    frame = model.part("front_frame")

    rim_geom = BezelGeometry(
        opening_size=(0.050, 0.030),
        outer_size=(0.064, 0.044),
        depth=0.006,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.006,
        outer_corner_radius=0.010,
    )
    rim_mesh = mesh_from_geometry(rim_geom, "lens_rim")
    lens_mesh = mesh_from_cadquery(_lens_plate_mesh(0.053, 0.033, 0.0022), "polycarbonate_lens")
    shield_mesh = mesh_from_cadquery(_side_shield_mesh(), "broad_side_shield")

    # Two framed front openings.  The bezel's local depth is rotated into world Y,
    # matching the thin lens plane.
    for side, x in (("left", -0.036), ("right", 0.036)):
        frame.visual(
            rim_mesh,
            origin=Origin(xyz=(x, 0.000, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black_frame,
            name=f"{side}_rim",
        )
        frame.visual(
            lens_mesh,
            origin=Origin(xyz=(x, -0.0005, 0.000)),
            material=clear_lens,
            name=f"{side}_lens",
        )

    # Bridge, brow, and nose pads connect the separate lens rims into a single
    # continuous front frame.
    frame.visual(
        Box((0.025, 0.007, 0.009)),
        origin=Origin(xyz=(0.0, 0.000, 0.007)),
        material=black_frame,
        name="bridge",
    )
    frame.visual(
        Box((0.150, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.000, 0.025)),
        material=black_frame,
        name="brow_bar",
    )
    for side, x in (("left", -0.012), ("right", 0.012)):
        frame.visual(
            Box((0.007, 0.004, 0.018)),
            origin=Origin(xyz=(x, 0.004, -0.020), rpy=(0.0, 0.0, 0.25 if side == "left" else -0.25)),
            material=Material(f"{side}_soft_nose_pad", rgba=(0.85, 0.95, 1.0, 0.55)),
            name=f"{side}_nose_pad",
        )

    # Broad, transparent side shields are fixed to the front frame just inboard
    # of the hinge knuckles so the temple arms can swing outside them.
    for side, sx in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            shield_mesh,
            origin=Origin(xyz=(sx * 0.073, 0.004, 0.000)),
            material=clear_lens,
            name=f"{side}_side_shield",
        )
        frame.visual(
            Box((0.022, 0.010, 0.028)),
            origin=Origin(xyz=(sx * 0.073, 0.001, 0.000)),
            material=black_frame,
            name=f"{side}_hinge_lug",
        )
        # Alternating parent hinge knuckles: upper and lower barrels.
        frame.visual(
            Cylinder(radius=0.0031, length=0.011),
            origin=Origin(xyz=(sx * 0.083, 0.000, 0.0115)),
            material=black_frame,
            name=f"{side}_upper_knuckle",
        )
        frame.visual(
            Cylinder(radius=0.0031, length=0.011),
            origin=Origin(xyz=(sx * 0.083, 0.000, -0.0115)),
            material=black_frame,
            name=f"{side}_lower_knuckle",
        )
        frame.visual(
            Cylinder(radius=0.0011, length=0.034),
            origin=Origin(xyz=(sx * 0.083, 0.000, 0.000)),
            material=metal,
            name=f"{side}_hinge_pin",
        )

    sleeve_mesh = mesh_from_cadquery(_sleeve_mesh(), "temple_sleeve_shell")

    def make_temple(side: str, sx: float) -> None:
        temple = model.part(f"{side}_temple")
        rear = model.part(f"{side}_rear_temple")

        # A middle knuckle interleaves with the two front-frame knuckles; the
        # narrow leaf bridges it into the sliding sleeve.
        temple.visual(
            Cylinder(radius=0.0029, length=0.010),
            origin=Origin(xyz=(0.0, 0.000, 0.000)),
            material=temple_plastic,
            name="middle_knuckle",
        )
        temple.visual(
            Box((0.008, 0.026, 0.012)),
            origin=Origin(xyz=(0.0, 0.012, 0.000)),
            material=temple_plastic,
            name="hinge_leaf",
        )
        temple.visual(
            sleeve_mesh,
            origin=Origin(xyz=(0.0, 0.057, 0.000)),
            material=temple_plastic,
            name="sleeve_shell",
        )
        # Raised clip ribs on the outside show the indexed telescoping detent.
        for i, y in enumerate((0.030, 0.043, 0.056, 0.069, 0.082)):
            temple.visual(
                Box((0.0108, 0.0020, 0.0022)),
                origin=Origin(xyz=(0.0, y, 0.0060)),
                material=black_frame,
                name=f"detent_rib_{i}",
            )

        rear.visual(
            Box((0.0060, 0.110, 0.0032)),
            # Extends forward of the slide joint so it remains retained inside
            # the sleeve even at maximum adjustment.
            origin=Origin(xyz=(0.0, 0.010, 0.000)),
            material=temple_plastic,
            name="slide_bar",
        )
        rear.visual(
            Box((0.0048, 0.022, 0.0050)),
            origin=Origin(xyz=(0.0, 0.060, -0.0005)),
            material=rubber,
            name="straight_tip",
        )
        rear.visual(
            Box((0.0048, 0.030, 0.0050)),
            origin=Origin(xyz=(0.0, 0.076, -0.008), rpy=(0.45, 0.0, 0.0)),
            material=rubber,
            name="ear_hook",
        )
        rear.visual(
            Box((0.0050, 0.014, 0.0080)),
            origin=Origin(xyz=(0.0, 0.067, -0.004)),
            material=rubber,
            name="tip_bridge",
        )
        rear.visual(
            Box((0.0048, 0.010, 0.0050)),
            origin=Origin(xyz=(0.0, 0.088, -0.018), rpy=(0.85, 0.0, 0.0)),
            material=rubber,
            name="hook_end",
        )
        rear.visual(
            Box((0.0050, 0.014, 0.0100)),
            origin=Origin(xyz=(0.0, 0.083, -0.013)),
            material=rubber,
            name="end_bridge",
        )

        hinge = model.articulation(
            f"{side}_temple_hinge",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=temple,
            origin=Origin(xyz=(sx * 0.083, 0.000, 0.000)),
            axis=(0.0, 0.0, sx),
            motion_limits=MotionLimits(effort=0.8, velocity=2.5, lower=0.0, upper=1.75),
        )
        hinge.meta["role"] = "temple fold hinge"

        slide = model.articulation(
            f"{side}_temple_slide",
            ArticulationType.PRISMATIC,
            parent=temple,
            child=rear,
            origin=Origin(xyz=(0.0, 0.098, 0.000)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.12, lower=0.0, upper=0.030),
        )
        slide.meta["role"] = "telescoping fit adjustment"

    make_temple("left", -1.0)
    make_temple("right", 1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for side in ("left", "right"):
        temple = object_model.get_part(f"{side}_temple")
        rear = object_model.get_part(f"{side}_rear_temple")
        hinge = object_model.get_articulation(f"{side}_temple_hinge")
        slide = object_model.get_articulation(f"{side}_temple_slide")

        ctx.expect_within(
            rear,
            temple,
            axes="xz",
            inner_elem="slide_bar",
            outer_elem="sleeve_shell",
            margin=0.001,
            name=f"{side} rear section fits within sleeve cross-section",
        )
        ctx.expect_overlap(
            rear,
            temple,
            axes="y",
            elem_a="slide_bar",
            elem_b="sleeve_shell",
            min_overlap=0.040,
            name=f"{side} collapsed rear section is deeply retained",
        )

        rest_pos = ctx.part_world_position(rear)
        with ctx.pose({slide: 0.030}):
            ctx.expect_within(
                rear,
                temple,
                axes="xz",
                inner_elem="slide_bar",
                outer_elem="sleeve_shell",
                margin=0.001,
                name=f"{side} extended rear section stays clipped in sleeve",
            )
            ctx.expect_overlap(
                rear,
                temple,
                axes="y",
                elem_a="slide_bar",
                elem_b="sleeve_shell",
                min_overlap=0.012,
                name=f"{side} extended rear section retains insertion",
            )
            extended_pos = ctx.part_world_position(rear)

        ctx.check(
            f"{side} temple adjustment slides rearward",
            rest_pos is not None and extended_pos is not None and extended_pos[1] > rest_pos[1] + 0.025,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

        straight_aabb = ctx.part_element_world_aabb(temple, elem="sleeve_shell")
        with ctx.pose({hinge: 1.20}):
            folded_aabb = ctx.part_element_world_aabb(temple, elem="sleeve_shell")
        straight_x = None if straight_aabb is None else (straight_aabb[0][0] + straight_aabb[1][0]) / 2.0
        folded_x = None if folded_aabb is None else (folded_aabb[0][0] + folded_aabb[1][0]) / 2.0
        if side == "left":
            folded_inward = folded_x is not None and straight_x is not None and folded_x > straight_x + 0.020
        else:
            folded_inward = folded_x is not None and straight_x is not None and folded_x < straight_x - 0.020
        ctx.check(
            f"{side} temple hinge folds inward",
            folded_inward,
            details=f"straight_x={straight_x}, folded_x={folded_x}",
        )

    return ctx.report()


object_model = build_object_model()
