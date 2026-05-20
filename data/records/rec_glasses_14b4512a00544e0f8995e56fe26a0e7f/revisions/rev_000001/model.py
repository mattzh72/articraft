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
)


FRAME_CENTER_Z = 0.030
HINGE_Y = -0.0063
RIGHT_HINGE_X = 0.0736
LEFT_HINGE_X = -RIGHT_HINGE_X
SPRING_REAR_Y = -0.0240


def _side_axis(side: int) -> tuple[float, float, float]:
    """Positive joint motion folds/deflects a temple toward the lens center."""
    return (0.0, 0.0, -1.0 if side > 0 else 1.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_spring_hinge_glasses")

    frame_mat = Material("dark_tortoise_acetate", rgba=(0.055, 0.038, 0.026, 1.0))
    brow_mat = Material("black_brow_acetate", rgba=(0.010, 0.009, 0.008, 1.0))
    lens_mat = Material("slightly_blue_lens", rgba=(0.72, 0.90, 1.00, 0.32))
    metal_mat = Material("brushed_silver", rgba=(0.72, 0.70, 0.66, 1.0))
    rubber_mat = Material("dark_ear_tip", rgba=(0.025, 0.022, 0.020, 1.0))

    frame = model.part("front_frame")

    # Dimensions in meters: a full-size adult pair of rectangular office glasses.
    lens_centers = (-0.033, 0.033)
    opening_w = 0.050
    opening_h = 0.030
    rim = 0.004
    frame_depth = 0.004

    for index, cx in enumerate(lens_centers):
        side_name = f"lens_{index}"
        # Seated transparent lenses, slightly oversized into the rim groove so the
        # lens reads captured by the frame rather than floating in the aperture.
        frame.visual(
            Box((opening_w + 0.002, 0.0014, opening_h + 0.002)),
            origin=Origin(xyz=(cx, 0.0002, FRAME_CENTER_Z)),
            material=lens_mat,
            name=f"{side_name}_glass",
        )
        # Rectangular rim members. The top members are intentionally a little
        # heavier and dead straight, giving the requested office-frame brow line.
        frame.visual(
            Box((opening_w + 2 * rim, frame_depth, rim * 1.18)),
            origin=Origin(
                xyz=(cx, 0.0, FRAME_CENTER_Z + opening_h / 2 + rim / 2)
            ),
            material=brow_mat,
            name=f"{side_name}_brow",
        )
        frame.visual(
            Box((opening_w + 2 * rim, frame_depth, rim)),
            origin=Origin(
                xyz=(cx, 0.0, FRAME_CENTER_Z - opening_h / 2 - rim / 2)
            ),
            material=frame_mat,
            name=f"{side_name}_lower_rim",
        )
        for edge, dx in (("inner", -1 if cx > 0 else 1), ("outer", 1 if cx > 0 else -1)):
            frame.visual(
                Box((rim, frame_depth, opening_h + 2 * rim)),
                origin=Origin(xyz=(cx + dx * (opening_w / 2 + rim / 2), 0.0, FRAME_CENTER_Z)),
                material=frame_mat,
                name=f"{side_name}_{edge}_rim",
            )

    # Two-piece bridge: a straight upper bridge aligned with the brows and a
    # smaller saddle below it so the nose opening remains visibly open.
    frame.visual(
        Box((0.018, frame_depth, 0.0040)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_CENTER_Z + 0.010)),
        material=brow_mat,
        name="straight_bridge",
    )
    frame.visual(
        Box((0.012, frame_depth * 0.85, 0.0032)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_CENTER_Z - 0.006)),
        material=frame_mat,
        name="lower_bridge",
    )

    # Side hinge blocks and their fixed outer knuckles. Small web plates tie the
    # knuckles back to the rectangular frame without using hidden floating pins.
    for side, hinge_x in ((1, RIGHT_HINGE_X), (-1, LEFT_HINGE_X)):
        suffix = "0" if side > 0 else "1"
        block_x = side * 0.06675
        web_x = side * 0.0705
        frame.visual(
            Box((0.0095, 0.0060, 0.020)),
            origin=Origin(xyz=(block_x, -0.0010, FRAME_CENTER_Z)),
            material=brow_mat,
            name=f"hinge_block_{suffix}",
        )
        frame.visual(
            Box((0.0020, 0.0025, 0.018)),
            origin=Origin(xyz=(web_x, -0.0052, FRAME_CENTER_Z)),
            material=metal_mat,
            name=f"hinge_web_{suffix}",
        )
        for level, dz in (("lower", -0.0060), ("upper", 0.0060)):
            frame.visual(
                Cylinder(radius=0.0021, length=0.0060),
                origin=Origin(xyz=(hinge_x, HINGE_Y, FRAME_CENTER_Z + dz)),
                material=metal_mat,
                name=f"frame_{level}_knuckle_{suffix}",
            )

    # Articulated spring segment plus long temple arm on each side. At q=0 both
    # temples extend straight backward; positive temple motion folds each arm
    # inward across the lenses.
    for side, hinge_x in ((1, RIGHT_HINGE_X), (-1, LEFT_HINGE_X)):
        suffix = "0" if side > 0 else "1"
        spring = model.part(f"spring_{suffix}")
        spring.visual(
            Cylinder(radius=0.00185, length=0.0060),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=metal_mat,
            name="side_center_knuckle",
        )
        spring.visual(
            Box((0.0042, 0.0216, 0.0034)),
            origin=Origin(xyz=(0.0, -0.0120, 0.0)),
            material=metal_mat,
            name="spring_body",
        )
        # Dark transverse bands make the short over-travel spring segment legible.
        for band_i, band_y in enumerate((-0.0075, -0.0115, -0.0155)):
            spring.visual(
                Box((0.0046, 0.0008, 0.0039)),
                origin=Origin(xyz=(0.0, band_y, 0.0)),
                material=brow_mat,
                name=f"spring_band_{band_i}",
            )
        spring.visual(
            Box((0.0038, 0.0023, 0.0180)),
            origin=Origin(xyz=(0.0, -0.02095, 0.0)),
            material=metal_mat,
            name="rear_yoke",
        )
        spring.visual(
            Cylinder(radius=0.0020, length=0.0060),
            origin=Origin(xyz=(0.0, SPRING_REAR_Y, -0.0060)),
            material=metal_mat,
            name="rear_lower_knuckle",
        )
        spring.visual(
            Cylinder(radius=0.0020, length=0.0060),
            origin=Origin(xyz=(0.0, SPRING_REAR_Y, 0.0060)),
            material=metal_mat,
            name="rear_upper_knuckle",
        )

        model.articulation(
            f"frame_to_spring_{suffix}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=spring,
            origin=Origin(xyz=(hinge_x, HINGE_Y, FRAME_CENTER_Z)),
            axis=_side_axis(side),
            motion_limits=MotionLimits(
                effort=0.35, velocity=2.5, lower=-0.28, upper=0.35
            ),
        )

        temple = model.part(f"temple_{suffix}")
        temple.visual(
            Cylinder(radius=0.00175, length=0.0060),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=metal_mat,
            name="temple_center_knuckle",
        )
        temple.visual(
            Box((0.0046, 0.103, 0.0034)),
            origin=Origin(xyz=(0.0, -0.0528, 0.0)),
            material=frame_mat,
            name="main_arm",
        )
        temple.visual(
            Box((0.0042, 0.036, 0.0038)),
            origin=Origin(xyz=(0.0, -0.1190, -0.0067), rpy=(0.38, 0.0, 0.0)),
            material=rubber_mat,
            name="ear_tip",
        )

        model.articulation(
            f"spring_to_temple_{suffix}",
            ArticulationType.REVOLUTE,
            parent=spring,
            child=temple,
            origin=Origin(xyz=(0.0, SPRING_REAR_Y, 0.0)),
            axis=_side_axis(side),
            motion_limits=MotionLimits(
                effort=0.25, velocity=2.2, lower=0.0, upper=1.70
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("front_frame")
    spring_0 = object_model.get_part("spring_0")
    spring_1 = object_model.get_part("spring_1")
    temple_0 = object_model.get_part("temple_0")
    temple_1 = object_model.get_part("temple_1")
    spring_joint_0 = object_model.get_articulation("frame_to_spring_0")
    spring_joint_1 = object_model.get_articulation("frame_to_spring_1")
    temple_joint_0 = object_model.get_articulation("spring_to_temple_0")
    temple_joint_1 = object_model.get_articulation("spring_to_temple_1")

    ctx.expect_gap(
        frame,
        temple_0,
        axis="y",
        min_gap=0.010,
        name="open temple_0 sits behind the frame",
    )
    ctx.expect_gap(
        frame,
        temple_1,
        axis="y",
        min_gap=0.010,
        name="open temple_1 sits behind the frame",
    )
    ctx.expect_overlap(
        spring_0,
        temple_0,
        axes="xy",
        elem_a="rear_lower_knuckle",
        elem_b="temple_center_knuckle",
        min_overlap=0.002,
        name="temple_0 hinge knuckles share a vertical pin line",
    )
    ctx.expect_contact(
        spring_0,
        temple_0,
        elem_a="rear_lower_knuckle",
        elem_b="temple_center_knuckle",
        contact_tol=0.00005,
        name="temple_0 interleaved knuckles touch at the hinge stack",
    )
    ctx.expect_overlap(
        spring_1,
        temple_1,
        axes="xy",
        elem_a="rear_upper_knuckle",
        elem_b="temple_center_knuckle",
        min_overlap=0.002,
        name="temple_1 hinge knuckles share a vertical pin line",
    )
    ctx.expect_contact(
        spring_1,
        temple_1,
        elem_a="rear_upper_knuckle",
        elem_b="temple_center_knuckle",
        contact_tol=0.00005,
        name="temple_1 interleaved knuckles touch at the hinge stack",
    )

    def _elem_center_x(part, elem_name: str) -> float | None:
        box = ctx.part_element_world_aabb(part, elem=elem_name)
        if box is None:
            return None
        lo, hi = box
        return (lo[0] + hi[0]) * 0.5

    spring_0_rest_x = _elem_center_x(spring_0, "spring_body")
    spring_1_rest_x = _elem_center_x(spring_1, "spring_body")
    with ctx.pose({spring_joint_0: 0.28, spring_joint_1: 0.28}):
        spring_0_flex_x = _elem_center_x(spring_0, "spring_body")
        spring_1_flex_x = _elem_center_x(spring_1, "spring_body")
    ctx.check(
        "short spring segments rotate independently",
        spring_0_rest_x is not None
        and spring_1_rest_x is not None
        and spring_0_flex_x is not None
        and spring_1_flex_x is not None
        and spring_0_flex_x < spring_0_rest_x - 0.002
        and spring_1_flex_x > spring_1_rest_x + 0.002,
        details=(
            f"spring_0 rest/flex={spring_0_rest_x}/{spring_0_flex_x}, "
            f"spring_1 rest/flex={spring_1_rest_x}/{spring_1_flex_x}"
        ),
    )

    temple_0_rest_x = _elem_center_x(temple_0, "main_arm")
    temple_1_rest_x = _elem_center_x(temple_1, "main_arm")
    with ctx.pose({temple_joint_0: 1.45, temple_joint_1: 1.45}):
        temple_0_fold_x = _elem_center_x(temple_0, "main_arm")
        temple_1_fold_x = _elem_center_x(temple_1, "main_arm")
    ctx.check(
        "temple arms fold inward on their side hinges",
        temple_0_rest_x is not None
        and temple_1_rest_x is not None
        and temple_0_fold_x is not None
        and temple_1_fold_x is not None
        and temple_0_fold_x < temple_0_rest_x - 0.018
        and temple_1_fold_x > temple_1_rest_x + 0.018,
        details=(
            f"temple_0 rest/fold={temple_0_rest_x}/{temple_0_fold_x}, "
            f"temple_1 rest/fold={temple_1_rest_x}/{temple_1_fold_x}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
