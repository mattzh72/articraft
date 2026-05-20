from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PANEL_HEIGHT = 1.35
PANEL_WIDTH = 0.52
PANEL_THICKNESS = 0.045
HINGE_X = 0.535
HINGE_Y = -0.035
LOUVER_ZS = (
    -0.52,
    -0.42,
    -0.32,
    -0.22,
    -0.12,
    0.12,
    0.22,
    0.32,
    0.42,
    0.52,
)


def _add_cylinder_head(part, x: float, y: float, z: float, name: str, material) -> None:
    """Small black screw or rivet head lying on the front face."""
    part.visual(
        Cylinder(radius=0.0065, length=0.003),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_shutter_frame(part, sign: float, paint, hardware) -> None:
    """Frame, rails, strap hinges, and face-mounted guide blocks for one leaf."""
    # Structural shutter rails and stiles.  Each member overlaps at the corners
    # so the leaf compiles as one connected joinery assembly.
    part.visual(
        Box((0.050, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(sign * 0.040, 0.0, 0.0)),
        material=paint,
        name="outer_stile",
    )
    part.visual(
        Box((0.070, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(sign * (PANEL_WIDTH - 0.035), 0.0, 0.0)),
        material=paint,
        name="meeting_stile",
    )
    part.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, 0.078)),
        origin=Origin(xyz=(sign * PANEL_WIDTH / 2.0, 0.0, PANEL_HEIGHT / 2.0 - 0.039)),
        material=paint,
        name="top_rail",
    )
    part.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, 0.078)),
        origin=Origin(xyz=(sign * PANEL_WIDTH / 2.0, 0.0, -PANEL_HEIGHT / 2.0 + 0.039)),
        material=paint,
        name="bottom_rail",
    )
    part.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, 0.060)),
        origin=Origin(xyz=(sign * PANEL_WIDTH / 2.0, 0.0, 0.0)),
        material=paint,
        name="lock_rail",
    )

    # Raised inner stops around each louver bay make the frame read like a real
    # routed plantation-shutter panel rather than a flat rectangle.
    for zc, suffix in ((0.3335, "upper"), (-0.3335, "lower")):
        part.visual(
            Box((0.018, 0.020, 0.607)),
            origin=Origin(xyz=(sign * 0.086, -0.027, zc)),
            material=paint,
            name=f"{suffix}_outer_stop",
        )
        part.visual(
            Box((0.018, 0.020, 0.607)),
            origin=Origin(xyz=(sign * 0.447, -0.027, zc)),
            material=paint,
            name=f"{suffix}_inner_stop",
        )

    # Stout top/bottom guides hold the adjustable tilt rod against the front of
    # the leaf; the moving rod slides through these ears.
    for zc, suffix in ((0.622, "upper"), (-0.622, "lower")):
        part.visual(
            Box((0.055, 0.050, 0.032)),
            origin=Origin(xyz=(sign * 0.260, -0.045, zc)),
            material=paint,
            name=f"{suffix}_tilt_guide",
        )

    # Decorative exterior strap hinges mounted on the front face.
    for zc, suffix in ((0.47, "upper"), (-0.47, "lower")):
        part.visual(
            Box((0.070, 0.009, 0.034)),
            origin=Origin(xyz=(sign * 0.040, -0.0265, zc)),
            material=hardware,
            name=f"{suffix}_strap",
        )
        part.visual(
            Box((0.050, 0.009, 0.054)),
            origin=Origin(xyz=(sign * 0.025, -0.0265, zc)),
            material=hardware,
            name=f"{suffix}_strap_leaf",
        )
        _add_cylinder_head(
            part,
            sign * 0.020,
            -0.0315,
            zc + 0.012,
            f"{suffix}_strap_screw_0",
            hardware,
        )
        _add_cylinder_head(
            part,
            sign * 0.043,
            -0.0315,
            zc,
            f"{suffix}_strap_screw_1",
            hardware,
        )
        _add_cylinder_head(
            part,
            sign * 0.064,
            -0.0315,
            zc,
            f"{suffix}_strap_screw_2",
            hardware,
        )
        # Moving hinge knuckle: the stationary root knuckles are staggered above
        # and below this barrel.
        part.visual(
            Cylinder(radius=0.013, length=0.052),
            origin=Origin(xyz=(0.0, -0.014, zc)),
            material=hardware,
            name=f"{suffix}_moving_knuckle",
        )

    # Small meeting-latch catch plates on the face, visually seated into the
    # lock rail but not modeled as a separate latch mechanism.
    part.visual(
        Box((0.034, 0.008, 0.060)),
        origin=Origin(xyz=(sign * (PANEL_WIDTH - 0.055), -0.0265, 0.0)),
        material=hardware,
        name="latch_plate",
    )


def _add_louver(part, paint, name_prefix: str) -> None:
    """One broad elliptical-looking slat with subtle top and bottom beads."""
    part.visual(
        Box((0.330, 0.018, 0.078)),
        origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(0.38, 0.0, 0.0)),
        material=paint,
        name=f"{name_prefix}_blade",
    )
    # Narrow beads on the long edges read as rounded wood profiles while staying
    # connected to the blade for island-free geometry.
    part.visual(
        Box((0.326, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.040, 0.034), rpy=(0.38, 0.0, 0.0)),
        material=paint,
        name=f"{name_prefix}_top_bead",
    )
    part.visual(
        Box((0.326, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.012, -0.034), rpy=(0.38, 0.0, 0.0)),
        material=paint,
        name=f"{name_prefix}_bottom_bead",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_louvered_shutter")

    painted = model.material("painted_sage_green", color=(0.19, 0.34, 0.25, 1.0))
    slat_paint = model.material("slightly_worn_slat_paint", color=(0.24, 0.42, 0.32, 1.0))
    trim = model.material("cream_exterior_trim", color=(0.82, 0.78, 0.67, 1.0))
    dark_recess = model.material("shadowed_window_recess", color=(0.035, 0.045, 0.052, 1.0))
    glass = model.material("muted_glass", color=(0.18, 0.28, 0.34, 0.65))
    black_iron = model.material("black_powder_coated_iron", color=(0.01, 0.01, 0.009, 1.0))

    surround = model.part("surround")
    # A shallow recessed architectural surround provides the fixed mounting
    # body, side jambs for the hinge plates, and the shadowed window behind the
    # shutters.  The trim members are intentionally in one root part because
    # they are a fixed built-in frame.
    surround.visual(
        Box((1.34, 0.035, 1.56)),
        origin=Origin(xyz=(0.0, 0.031, 0.0)),
        material=dark_recess,
        name="shadow_recess",
    )
    surround.visual(
        Box((0.92, 0.012, 1.10)),
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material=glass,
        name="dark_glass",
    )
    surround.visual(
        Box((1.34, 0.058, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=trim,
        name="top_trim",
    )
    surround.visual(
        Box((1.34, 0.058, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, -0.735)),
        material=trim,
        name="bottom_sill",
    )
    surround.visual(
        Box((0.095, 0.058, 1.56)),
        origin=Origin(xyz=(-0.617, 0.0, 0.0)),
        material=trim,
        name="side_trim_0",
    )
    surround.visual(
        Box((0.095, 0.058, 1.56)),
        origin=Origin(xyz=(0.617, 0.0, 0.0)),
        material=trim,
        name="side_trim_1",
    )
    surround.visual(
        Box((0.050, 0.028, 1.38)),
        origin=Origin(xyz=(-0.570, -0.018, 0.0)),
        material=trim,
        name="hinge_jamb_0",
    )
    surround.visual(
        Box((0.050, 0.028, 1.38)),
        origin=Origin(xyz=(0.570, -0.018, 0.0)),
        material=trim,
        name="hinge_jamb_1",
    )

    # Fixed hinge leaves, barrels, and screw heads on the surround.
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        x = sign * HINGE_X
        leaf_x = sign * 0.570
        for zc, suffix in ((0.47, "upper"), (-0.47, "lower")):
            surround.visual(
                Box((0.044, 0.008, 0.154)),
                origin=Origin(xyz=(leaf_x, -0.031, zc)),
                material=black_iron,
                name=f"{side_name}_{suffix}_fixed_leaf",
            )
            for dz, k in ((0.045, "top"), (-0.045, "bottom")):
                surround.visual(
                    Cylinder(radius=0.013, length=0.040),
                    origin=Origin(xyz=(x, HINGE_Y, zc + dz)),
                    material=black_iron,
                    name=f"{side_name}_{suffix}_{k}_fixed_knuckle",
                )
            _add_cylinder_head(surround, leaf_x, -0.0355, zc + 0.045, f"{side_name}_{suffix}_screw_0", black_iron)
            _add_cylinder_head(surround, leaf_x, -0.0355, zc - 0.045, f"{side_name}_{suffix}_screw_1", black_iron)

        # Static exterior holdback dogs on the trim read as believable shutter
        # hardware without adding a secondary mechanism to the main product.
        surround.visual(
            Box((0.090, 0.014, 0.020)),
            origin=Origin(xyz=(sign * 0.690, -0.030, -0.514), rpy=(0.0, 0.0, sign * 0.65)),
            material=black_iron,
            name=f"{side_name}_holdback_arm",
        )
        surround.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(sign * 0.667, -0.030, -0.514), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_iron,
            name=f"{side_name}_holdback_pivot",
        )

    left_shutter = model.part("left_shutter")
    right_shutter = model.part("right_shutter")
    _add_shutter_frame(left_shutter, 1.0, painted, black_iron)
    _add_shutter_frame(right_shutter, -1.0, painted, black_iron)

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=left_shutter,
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=right_shutter,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=0.0, upper=1.75),
    )

    for side_name, parent, sign in (
        ("left", left_shutter, 1.0),
        ("right", right_shutter, -1.0),
    ):
        # The vertical tilt bar is the user-facing louver adjustment.  Sliding
        # it a few centimeters raises/lowers the front edge of every slat.
        tilt_bar = model.part(f"{side_name}_tilt_bar")
        tilt_bar.visual(
            Box((0.018, 0.014, 1.240)),
            origin=Origin(),
            material=black_iron,
            name="vertical_bar",
        )
        for i, z in enumerate(LOUVER_ZS):
            tilt_bar.visual(
                Box((0.036, 0.018, 0.012)),
                origin=Origin(xyz=(0.0, 0.008, z)),
                material=black_iron,
                name=f"staple_tab_{i}",
            )
        tilt_joint_name = f"{side_name}_tilt_slide"
        model.articulation(
            tilt_joint_name,
            ArticulationType.PRISMATIC,
            parent=parent,
            child=tilt_bar,
            origin=Origin(xyz=(sign * 0.260, -0.058, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.25, lower=-0.055, upper=0.055),
        )

        for i, z in enumerate(LOUVER_ZS):
            louver = model.part(f"{side_name}_slat_{i}")
            _add_louver(louver, slat_paint, "slat")
            mimic = None
            if i > 0:
                mimic = Mimic(joint=f"{side_name}_slat_0_pivot")
            model.articulation(
                f"{side_name}_slat_{i}_pivot",
                ArticulationType.REVOLUTE,
                parent=parent,
                child=louver,
                origin=Origin(xyz=(sign * 0.260, 0.0, z)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-0.60, upper=0.60),
                mimic=mimic,
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    surround = object_model.get_part("surround")
    left = object_model.get_part("left_shutter")
    right = object_model.get_part("right_shutter")
    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")
    left_tilt = object_model.get_articulation("left_tilt_slide")
    left_louver = object_model.get_articulation("left_slat_0_pivot")

    for side in ("left", "right"):
        leaf = object_model.get_part(f"{side}_shutter")
        tilt_bar = object_model.get_part(f"{side}_tilt_bar")
        for guide in ("upper_tilt_guide", "lower_tilt_guide"):
            ctx.allow_overlap(
                leaf,
                tilt_bar,
                elem_a=guide,
                elem_b="vertical_bar",
                reason="The tilt rod is intentionally captured through a painted guide ear so it can slide without floating off the shutter.",
            )
            ctx.expect_overlap(
                leaf,
                tilt_bar,
                axes="xy",
                min_overlap=0.010,
                elem_a=guide,
                elem_b="vertical_bar",
                name=f"{side} tilt rod is retained by {guide}",
            )

    ctx.expect_overlap(
        left,
        surround,
        axes="z",
        min_overlap=1.20,
        elem_a="outer_stile",
        elem_b="hinge_jamb_0",
        name="left leaf spans the mounted hinge jamb",
    )
    ctx.expect_overlap(
        right,
        surround,
        axes="z",
        min_overlap=1.20,
        elem_a="outer_stile",
        elem_b="hinge_jamb_1",
        name="right leaf spans the mounted hinge jamb",
    )
    ctx.expect_gap(
        right,
        left,
        axis="x",
        max_gap=0.040,
        max_penetration=0.0,
        positive_elem="meeting_stile",
        negative_elem="meeting_stile",
        name="closed shutters meet with a narrow center reveal",
    )

    rest_left_box = ctx.part_world_aabb(left)
    rest_right_box = ctx.part_world_aabb(right)
    with ctx.pose({left_hinge: 1.20, right_hinge: 1.20}):
        open_left_box = ctx.part_world_aabb(left)
        open_right_box = ctx.part_world_aabb(right)
        ctx.expect_gap(
            right,
            left,
            axis="x",
            min_gap=0.080,
            positive_elem="meeting_stile",
            negative_elem="meeting_stile",
            name="opened leaves swing away from the center",
        )

    ctx.check(
        "leaf hinges swing outward from the facade",
        rest_left_box is not None
        and rest_right_box is not None
        and open_left_box is not None
        and open_right_box is not None
        and open_left_box[0][1] < rest_left_box[0][1] - 0.15
        and open_right_box[0][1] < rest_right_box[0][1] - 0.15,
        details=f"left {rest_left_box}->{open_left_box}, right {rest_right_box}->{open_right_box}",
    )

    slat = object_model.get_part("left_slat_4")
    rest_slat_pos = ctx.part_world_aabb(slat)
    with ctx.pose({left_louver: 0.60, left_tilt: 0.055}):
        raised_slat_pos = ctx.part_world_aabb(slat)
        ctx.expect_overlap(
            slat,
            left,
            axes="x",
            min_overlap=0.30,
            elem_a="slat_blade",
            elem_b="lock_rail",
            name="adjusting louver remains captured across the panel width",
        )

    ctx.check(
        "tilt bar changes slat attitude",
        rest_slat_pos is not None
        and raised_slat_pos is not None
        and (raised_slat_pos[1][2] - raised_slat_pos[0][2]) > (rest_slat_pos[1][2] - rest_slat_pos[0][2]) + 0.006,
        details=f"rest_aabb={rest_slat_pos}, adjusted_aabb={raised_slat_pos}",
    )

    return ctx.report()


object_model = build_object_model()
