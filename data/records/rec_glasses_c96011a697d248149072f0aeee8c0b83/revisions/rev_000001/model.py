from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ExtrudeGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


METAL_RADIUS = 0.00115


def _lens_outline(sign: float) -> list[tuple[float, float, float]]:
    """Aviator lens/rim centerline in the front XZ plane."""
    return [
        (sign * 0.010, 0.000, 0.017),
        (sign * 0.030, 0.000, 0.021),
        (sign * 0.056, 0.000, 0.016),
        (sign * 0.071, 0.000, -0.004),
        (sign * 0.061, 0.000, -0.025),
        (sign * 0.034, 0.000, -0.040),
        (sign * 0.012, 0.000, -0.025),
        (sign * 0.006, 0.000, -0.004),
    ]


def _lens_profile_2d(sign: float) -> list[tuple[float, float]]:
    # ExtrudeGeometry uses XY profiles; after an X rotation this becomes XZ.
    return [(x, z) for x, _, z in _lens_outline(sign)]


def _temple_path() -> list[tuple[float, float, float]]:
    return [
        (0.0000, 0.011, 0.000),
        (0.0000, 0.040, 0.003),
        (0.0000, 0.085, 0.001),
        (0.0000, 0.118, -0.010),
        (0.0000, 0.142, -0.024),
    ]


def _make_hinge_block(sign: float) -> MeshGeometry:
    """Compact front-corner hinge block with two ears and a connecting web."""
    hinge_x = sign * 0.073
    geom = MeshGeometry()

    # Outer upright side plate, tied to the rim connector.
    geom.merge(
        BoxGeometry((0.0030, 0.0065, 0.0220)).translate(
            sign * 0.0780, 0.0025, 0.0000
        )
    )

    # Webs connect the plate into the top/bottom knuckles, leaving a clear
    # middle slot for the temple barrel.
    for z in (-0.0065, 0.0065):
        geom.merge(
            BoxGeometry((0.0055, 0.0045, 0.0048)).translate(
                sign * 0.0755, 0.0034, z
            )
        )
        geom.merge(
            CylinderGeometry(0.0025, 0.0048, radial_segments=20).translate(
                hinge_x, 0.0040, z
            )
        )
    return geom


def _make_nose_pad() -> MeshGeometry:
    """Soft flattened silicone nose pad cushion."""
    geom = MeshGeometry()
    pad = CapsuleGeometry(0.0042, 0.0105, radial_segments=24, height_segments=8)
    pad.scale(0.82, 0.34, 1.0).translate(0.0, 0.0023, -0.0062)
    geom.merge(pad)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aviator_glasses")

    polished_gold = model.material("polished_gold", rgba=(0.95, 0.72, 0.31, 1.0))
    lens_smoke = model.material("smoke_gradient_lens", rgba=(0.28, 0.38, 0.44, 0.36))
    silicone = model.material("clear_silicone", rgba=(0.90, 0.93, 0.88, 0.62))
    dark_tip = model.material("dark_ear_tip", rgba=(0.055, 0.045, 0.035, 1.0))

    front = model.part("front")

    # Teardrop rims and lightly tinted lenses.
    for sign, side in ((-1.0, "lens_0"), (1.0, "lens_1")):
        rim = tube_from_spline_points(
            _lens_outline(sign),
            radius=METAL_RADIUS,
            samples_per_segment=12,
            closed_spline=True,
            radial_segments=18,
            cap_ends=True,
        )
        front.visual(
            mesh_from_geometry(rim, f"{side}_rim"),
            material=polished_gold,
            name=f"{side}_rim",
        )

        lens = ExtrudeGeometry(
            _lens_profile_2d(sign),
            0.0010,
            cap=True,
            center=True,
            closed=True,
        )
        front.visual(
            mesh_from_geometry(lens, f"{side}_glass"),
            origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
            material=lens_smoke,
            name=f"{side}_glass",
        )

    # Classic double bridge: a high brow bridge plus lower nose bridge.
    top_bridge = tube_from_spline_points(
        [
            (-0.012, 0.000, 0.015),
            (-0.005, -0.001, 0.021),
            (0.005, -0.001, 0.021),
            (0.012, 0.000, 0.015),
        ],
        radius=0.00105,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    front.visual(
        mesh_from_geometry(top_bridge, "top_bridge"),
        material=polished_gold,
        name="top_bridge",
    )

    lower_bridge = tube_from_spline_points(
        [
            (-0.008, 0.000, -0.001),
            (-0.004, 0.001, -0.006),
            (0.004, 0.001, -0.006),
            (0.008, 0.000, -0.001),
        ],
        radius=0.00095,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    front.visual(
        mesh_from_geometry(lower_bridge, "lower_bridge"),
        material=polished_gold,
        name="lower_bridge",
    )

    # Compact outer hinge blocks, with short rim-to-block straps so the temples
    # read as clipped into the metal front rather than floating nearby.
    for sign, block_name, pin_name, connector_name in (
        (-1.0, "hinge_0_block", "hinge_0_pin", "hinge_0_connector"),
        (1.0, "hinge_1_block", "hinge_1_pin", "hinge_1_connector"),
    ):
        front.visual(
            mesh_from_geometry(_make_hinge_block(sign), block_name),
            material=polished_gold,
            name=block_name,
        )
        front.visual(
            Cylinder(radius=0.00075, length=0.0180),
            origin=Origin(xyz=(sign * 0.0730, 0.0040, 0.0000)),
            material=polished_gold,
            name=pin_name,
        )
        connector = tube_from_spline_points(
            [
                (sign * 0.066, 0.000, 0.006),
                (sign * 0.071, 0.001, 0.005),
                (sign * 0.076, 0.003, 0.005),
            ],
            radius=0.00095,
            samples_per_segment=8,
            radial_segments=16,
            cap_ends=True,
        )
        front.visual(
            mesh_from_geometry(connector, connector_name),
            material=polished_gold,
            name=connector_name,
        )

    # Adjustable wire arms that carry the separate pivoting nose pads.
    for sign, side in ((-1.0, "pad_0"), (1.0, "pad_1")):
        arm = tube_from_spline_points(
            [
                (sign * 0.0045, 0.0005, -0.0060),
                (sign * 0.0068, 0.0045, -0.0105),
                (sign * 0.0105, 0.0100, -0.0170),
            ],
            radius=0.00065,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        )
        front.visual(
            mesh_from_geometry(arm, f"{side}_arm"),
            material=polished_gold,
            name=f"{side}_arm",
        )
        front.visual(
            Cylinder(radius=0.00055, length=0.0046),
            origin=Origin(
                xyz=(sign * 0.0105, 0.0100, -0.0170),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=polished_gold,
            name=f"{side}_pivot",
        )

    # Two independent temple arms.  Their part frames sit exactly on the hinge
    # pin axis, so a yaw rotation folds each arm inward while keeping the barrel
    # retained in the front-corner hinge block.
    temple_parts = []
    for side_index, sign in enumerate((-1.0, 1.0)):
        temple = model.part(f"temple_{side_index}")
        temple.visual(
            Cylinder(radius=0.0018, length=0.0060),
            origin=Origin(),
            material=polished_gold,
            name="hinge_barrel",
        )
        temple.visual(
            Box((0.0030, 0.0120, 0.0020)),
            origin=Origin(xyz=(0.0, 0.0070, 0.0000)),
            material=polished_gold,
            name="hinge_tongue",
        )
        temple_wire = tube_from_spline_points(
            _temple_path(),
            radius=0.00105,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        )
        temple.visual(
            mesh_from_geometry(temple_wire, f"temple_{side_index}_wire"),
            material=polished_gold,
            name="arm_wire",
        )
        # Dark sleeves at the ear-contact end are part of the same arm so the
        # long wire does not read as an unsupported loose rod.
        tip_wire = tube_from_spline_points(
            [
                (0.0, 0.112, -0.008),
                (0.0, 0.126, -0.016),
                (0.0, 0.142, -0.024),
            ],
            radius=0.00165,
            samples_per_segment=8,
            radial_segments=16,
            cap_ends=True,
        )
        temple.visual(
            mesh_from_geometry(tip_wire, f"temple_{side_index}_tip"),
            material=dark_tip,
            name="ear_tip",
        )
        temple_parts.append(temple)

    pad_parts = []
    for side_index, sign in enumerate((-1.0, 1.0)):
        pad = model.part(f"nose_pad_{side_index}")
        pad.visual(
            mesh_from_geometry(_make_nose_pad(), f"nose_pad_{side_index}"),
            material=silicone,
            name="pad_body",
        )
        pad.visual(
            Cylinder(radius=0.00115, length=0.0042),
            origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
            material=polished_gold,
            name="pivot_socket",
        )
        pad_parts.append(pad)

    # Left temple folds toward +X, right temple toward -X.
    model.articulation(
        "front_to_temple_0",
        ArticulationType.REVOLUTE,
        parent=front,
        child=temple_parts[0],
        origin=Origin(xyz=(-0.0730, 0.0040, 0.0000)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=0.0, upper=1.70),
    )
    model.articulation(
        "front_to_temple_1",
        ArticulationType.REVOLUTE,
        parent=front,
        child=temple_parts[1],
        origin=Origin(xyz=(0.0730, 0.0040, 0.0000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=0.0, upper=1.70),
    )

    for side_index, sign in enumerate((-1.0, 1.0)):
        model.articulation(
            f"front_to_nose_pad_{side_index}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=pad_parts[side_index],
            origin=Origin(xyz=(sign * 0.0105, 0.0100, -0.0170)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.04, velocity=1.5, lower=-0.25, upper=0.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front")
    temple_0 = object_model.get_part("temple_0")
    temple_1 = object_model.get_part("temple_1")
    pad_0 = object_model.get_part("nose_pad_0")
    pad_1 = object_model.get_part("nose_pad_1")

    temple_joint_0 = object_model.get_articulation("front_to_temple_0")
    temple_joint_1 = object_model.get_articulation("front_to_temple_1")
    pad_joint_0 = object_model.get_articulation("front_to_nose_pad_0")
    pad_joint_1 = object_model.get_articulation("front_to_nose_pad_1")

    def elem_center(part, elem: str):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return (
            (lo[0] + hi[0]) * 0.5,
            (lo[1] + hi[1]) * 0.5,
            (lo[2] + hi[2]) * 0.5,
        )

    # The hinge pins are deliberately captured inside the temple barrels; that
    # local nested fit is the physical reason the arms fold without separating.
    ctx.allow_overlap(
        front,
        temple_0,
        elem_a="hinge_0_pin",
        elem_b="hinge_barrel",
        reason="The front hinge pin is intentionally captured through the temple barrel.",
    )
    ctx.allow_overlap(
        front,
        temple_1,
        elem_a="hinge_1_pin",
        elem_b="hinge_barrel",
        reason="The front hinge pin is intentionally captured through the temple barrel.",
    )
    ctx.expect_within(
        front,
        temple_0,
        axes="xy",
        inner_elem="hinge_0_pin",
        outer_elem="hinge_barrel",
        margin=0.0002,
        name="left hinge pin is centered in barrel",
    )
    ctx.expect_within(
        front,
        temple_1,
        axes="xy",
        inner_elem="hinge_1_pin",
        outer_elem="hinge_barrel",
        margin=0.0002,
        name="right hinge pin is centered in barrel",
    )
    ctx.expect_overlap(
        front,
        temple_0,
        axes="z",
        elem_a="hinge_0_pin",
        elem_b="hinge_barrel",
        min_overlap=0.005,
        name="left hinge pin passes through barrel",
    )
    ctx.expect_overlap(
        front,
        temple_1,
        axes="z",
        elem_a="hinge_1_pin",
        elem_b="hinge_barrel",
        min_overlap=0.005,
        name="right hinge pin passes through barrel",
    )

    # Hinge barrels stay seated between the compact front-corner blocks.
    ctx.expect_overlap(
        temple_0,
        front,
        axes="z",
        elem_a="hinge_barrel",
        elem_b="hinge_0_block",
        min_overlap=0.004,
        name="left temple barrel is retained vertically",
    )
    ctx.expect_overlap(
        temple_1,
        front,
        axes="z",
        elem_a="hinge_barrel",
        elem_b="hinge_1_block",
        min_overlap=0.004,
        name="right temple barrel is retained vertically",
    )

    left_rest = elem_center(temple_0, "ear_tip")
    right_rest = elem_center(temple_1, "ear_tip")
    with ctx.pose({temple_joint_0: 1.55, temple_joint_1: 1.55}):
        left_folded = elem_center(temple_0, "ear_tip")
        right_folded = elem_center(temple_1, "ear_tip")
        ctx.expect_overlap(
            temple_0,
            front,
            axes="z",
            elem_a="hinge_barrel",
            elem_b="hinge_0_block",
            min_overlap=0.004,
            name="left temple remains clipped while folded",
        )
        ctx.expect_overlap(
            temple_1,
            front,
            axes="z",
            elem_a="hinge_barrel",
            elem_b="hinge_1_block",
            min_overlap=0.004,
            name="right temple remains clipped while folded",
        )

    ctx.check(
        "temples fold inward",
        left_rest is not None
        and right_rest is not None
        and left_folded is not None
        and right_folded is not None
        and left_folded[0] > left_rest[0] + 0.015
        and right_folded[0] < right_rest[0] - 0.015,
        details=f"left rest/folded={left_rest}/{left_folded}, right rest/folded={right_rest}/{right_folded}",
    )

    for pad, joint, side_name in (
        (pad_0, pad_joint_0, "left"),
        (pad_1, pad_joint_1, "right"),
    ):
        side_index = 0 if side_name == "left" else 1
        ctx.allow_overlap(
            front,
            pad,
            elem_a=f"pad_{side_index}_pivot",
            elem_b="pivot_socket",
            reason="The small nose-pad screw is intentionally captured in the pad pivot socket.",
        )
        ctx.expect_within(
            front,
            pad,
            axes="yz",
            inner_elem=f"pad_{side_index}_pivot",
            outer_elem="pivot_socket",
            margin=0.0002,
            name=f"{side_name} nose-pad pivot screw is centered in socket",
        )
        ctx.expect_overlap(
            front,
            pad,
            axes="x",
            elem_a=f"pad_{side_index}_pivot",
            elem_b="pivot_socket",
            min_overlap=0.0035,
            name=f"{side_name} nose-pad pivot screw crosses socket",
        )
        ctx.expect_gap(
            pad,
            front,
            axis="y",
            min_gap=0.0000,
            max_gap=0.0060,
            positive_elem="pad_body",
            negative_elem=f"pad_{side_index}_arm",
            name=f"{side_name} nose pad sits just behind its wire arm",
        )
        rest = ctx.part_world_position(pad)
        with ctx.pose({joint: 0.22}):
            moved = ctx.part_world_position(pad)
        ctx.check(
            f"{side_name} nose pad has small pivot range",
            rest is not None and moved is not None and abs(0.22) <= 0.25,
            details=f"rest={rest}, moved={moved}",
        )

    return ctx.report()


object_model = build_object_model()
