from __future__ import annotations

import math

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
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_trunnion_module")

    painted = Material("satin_black_painted_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    edge_dark = Material("dark_machined_edges", rgba=(0.015, 0.017, 0.018, 1.0))
    shaft_metal = Material("brushed_trunnion_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    face_paint = Material("warm_gray_carried_face", rgba=(0.46, 0.48, 0.47, 1.0))
    glass = Material("smoked_inset_face", rgba=(0.03, 0.045, 0.055, 1.0))

    support = model.part("top_bracket")

    # A compact inverted yoke: the broad mounting plate is above, while the
    # bored cheeks drop down to carry the trunnion below the bracket.
    axis_z = 0.220
    yoke_top_z = 0.340
    yoke = TrunnionYokeGeometry(
        (0.360, 0.120, 0.200),
        span_width=0.270,
        trunnion_diameter=0.046,
        trunnion_center_z=yoke_top_z - axis_z,
        base_thickness=0.016,
        corner_radius=0.006,
        center=False,
    )
    support.visual(
        mesh_from_geometry(yoke, "inverted_trunnion_yoke"),
        origin=Origin(xyz=(0.0, 0.0, yoke_top_z), rpy=(math.pi, 0.0, 0.0)),
        material=painted,
        name="side_cheeks",
    )
    support.visual(
        Box((0.440, 0.190, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, yoke_top_z + 0.015)),
        material=painted,
        name="top_plate",
    )
    support.visual(
        Box((0.330, 0.026, 0.125)),
        origin=Origin(xyz=(0.0, 0.073, 0.260)),
        material=edge_dark,
        name="rear_web",
    )
    for i, x in enumerate((-0.160, 0.160)):
        for j, y in enumerate((-0.065, 0.065)):
            support.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(xyz=(x, y, yoke_top_z + 0.035)),
                material=shaft_metal,
                name=f"mount_bolt_{i}_{j}",
            )

    face = model.part("carried_face")
    face.visual(
        Cylinder(radius=0.0235, length=0.398),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_metal,
        name="trunnion_shaft",
    )
    face.visual(
        Cylinder(radius=0.034, length=0.138),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=edge_dark,
        name="center_hub",
    )
    for side, x in (("negative", -0.207), ("positive", 0.207)):
        face.visual(
            Cylinder(radius=0.026, length=0.016),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=shaft_metal,
            name=f"{side}_shaft_collar",
        )
    face.visual(
        Box((0.250, 0.028, 0.180)),
        origin=Origin(xyz=(0.0, -0.083, -0.130)),
        material=face_paint,
        name="face_plate",
    )
    face.visual(
        Box((0.170, 0.052, 0.035)),
        origin=Origin(xyz=(0.0, -0.055, -0.034)),
        material=edge_dark,
        name="hub_saddle",
    )
    for i, x in enumerate((-0.085, 0.085)):
        face.visual(
            Box((0.034, 0.044, 0.118)),
            origin=Origin(xyz=(x, -0.057, -0.074)),
            material=edge_dark,
            name=f"drop_arm_{i}",
        )
    face.visual(
        Box((0.205, 0.006, 0.112)),
        origin=Origin(xyz=(0.0, -0.100, -0.133)),
        material=glass,
        name="inset_panel",
    )
    face.visual(
        Box((0.262, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.103, -0.039)),
        material=edge_dark,
        name="top_lip",
    )
    face.visual(
        Box((0.262, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.103, -0.221)),
        material=edge_dark,
        name="bottom_lip",
    )
    for i, x in enumerate((-0.134, 0.134)):
        face.visual(
            Box((0.020, 0.012, 0.182)),
            origin=Origin(xyz=(x, -0.103, -0.130)),
            material=edge_dark,
            name=f"side_lip_{i}",
        )

    model.articulation(
        "pitch_trunnion",
        ArticulationType.REVOLUTE,
        parent=support,
        child=face,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.45, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_bracket")
    face = object_model.get_part("carried_face")
    joint = object_model.get_articulation("pitch_trunnion")

    ctx.allow_overlap(
        support,
        face,
        elem_a="side_cheeks",
        elem_b="trunnion_shaft",
        reason=(
            "The trunnion shaft is intentionally captured in the bored cheek "
            "bearing with a tiny interference fit so the under-slung member is "
            "visibly supported rather than floating."
        ),
    )

    ctx.expect_gap(
        support,
        face,
        axis="z",
        positive_elem="top_plate",
        negative_elem="face_plate",
        min_gap=0.080,
        name="carried face hangs below top support",
    )
    ctx.expect_overlap(
        support,
        face,
        axes="yz",
        elem_a="side_cheeks",
        elem_b="trunnion_shaft",
        min_overlap=0.020,
        name="shaft lies on cheek-supported trunnion axis",
    )

    rest_aabb = ctx.part_element_world_aabb(face, elem="face_plate")
    with ctx.pose({joint: 0.55}):
        raised_aabb = ctx.part_element_world_aabb(face, elem="face_plate")
        ctx.expect_gap(
            support,
            face,
            axis="z",
            positive_elem="top_plate",
            negative_elem="face_plate",
            min_gap=0.055,
            name="tilted face remains below top plate",
        )
    ctx.check(
        "pitch joint changes face attitude",
        rest_aabb is not None
        and raised_aabb is not None
        and abs(raised_aabb[0][1] - rest_aabb[0][1]) > 0.030,
        details=f"rest_aabb={rest_aabb}, raised_aabb={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
