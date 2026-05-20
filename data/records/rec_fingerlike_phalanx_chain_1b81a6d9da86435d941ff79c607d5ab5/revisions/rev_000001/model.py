from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PIN_AXIS_RPY = (math.pi / 2.0, 0.0, 0.0)
HINGE_AXIS = (0.0, -1.0, 0.0)


def _spatulate_pad_mesh():
    """A flattened rounded fingertip pad, wider than the phalanx spar."""
    profile = rounded_rect_profile(0.078, 0.052, radius=0.026, corner_segments=10)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, 0.010, center=True),
        "spatulate_fingertip_pad",
    )


def _add_clevis_yoke(part, *, x: float, material, prefix: str) -> None:
    """Add double side cheeks, a pin, and visible cap washers at one hinge."""
    cheek_y = 0.018
    cheek_thickness = 0.006
    part.visual(
        Box((0.070, cheek_thickness, 0.030)),
        origin=Origin(xyz=(x - 0.006, cheek_y, 0.0)),
        material=material,
        name=f"{prefix}_cheek_0",
    )
    part.visual(
        Box((0.070, cheek_thickness, 0.030)),
        origin=Origin(xyz=(x - 0.006, -cheek_y, 0.0)),
        material=material,
        name=f"{prefix}_cheek_1",
    )
    part.visual(
        Cylinder(radius=0.0042, length=0.052),
        origin=Origin(xyz=(x, 0.0, 0.0), rpy=PIN_AXIS_RPY),
        material=material,
        name=f"{prefix}_pin",
    )
    for side, y in enumerate((0.025, -0.025)):
        part.visual(
            Cylinder(radius=0.015, length=0.006),
            origin=Origin(xyz=(x, y, 0.0), rpy=PIN_AXIS_RPY),
            material=material,
            name=f"{prefix}_cap_{side}",
        )


def _add_distal_yoke(part, *, link_length: float, material) -> None:
    """Connect a link body to the next knuckle's two side cheeks."""
    part.visual(
        Box((0.022, 0.040, 0.014)),
        origin=Origin(xyz=(link_length - 0.042, 0.0, 0.0)),
        material=material,
        name="yoke_bridge",
    )
    _add_clevis_yoke(part, x=link_length, material=material, prefix="distal")


def _add_phalanx(
    part,
    *,
    link_length: float,
    material,
    barrel_material,
    has_distal_yoke: bool,
    fingertip_mesh=None,
    pad_material=None,
    ridge_material=None,
) -> None:
    """Build one narrow phalanx around a proximal hinge barrel."""
    part.visual(
        Cylinder(radius=0.0115, length=0.022),
        origin=Origin(rpy=PIN_AXIS_RPY),
        material=barrel_material,
        name="proximal_barrel",
    )

    body_end = link_length - (0.045 if has_distal_yoke else 0.006)
    body_start = 0.008
    body_length = body_end - body_start
    part.visual(
        Box((body_length, 0.016, 0.012)),
        origin=Origin(xyz=((body_start + body_end) * 0.5, 0.0, 0.0)),
        material=material,
        name="slender_spar",
    )
    part.visual(
        Box((body_length * 0.72, 0.004, 0.014)),
        origin=Origin(xyz=(body_start + body_length * 0.55, 0.0, 0.007)),
        material=barrel_material,
        name="top_rib",
    )

    if has_distal_yoke:
        _add_distal_yoke(part, link_length=link_length, material=material)

    if fingertip_mesh is not None and pad_material is not None:
        part.visual(
            Box((0.030, 0.020, 0.010)),
            origin=Origin(xyz=(link_length - 0.010, 0.0, 0.0)),
            material=material,
            name="pad_neck",
        )
        part.visual(
            fingertip_mesh,
            origin=Origin(xyz=(link_length + 0.026, 0.0, 0.0)),
            material=pad_material,
            name="fingertip_pad",
        )
        for index, dx in enumerate((0.008, 0.023, 0.038)):
            part.visual(
                Box((0.003, 0.038, 0.002)),
                origin=Origin(xyz=(link_length + dx, 0.0, 0.0054)),
                material=ridge_material or pad_material,
                name=f"tactile_ridge_{index}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_probing_finger")

    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_barrels = model.material("dark_hinge_barrels", rgba=(0.15, 0.16, 0.17, 1.0))
    graphite = model.material("graphite_handle", rgba=(0.12, 0.13, 0.14, 1.0))
    soft_pad = model.material("soft_blue_pad", rgba=(0.18, 0.30, 0.42, 1.0))
    pad_ridge = model.material("pad_ridge", rgba=(0.08, 0.13, 0.18, 1.0))

    pad_mesh = _spatulate_pad_mesh()

    base = model.part("base_mount")
    base.visual(
        Box((0.150, 0.026, 0.018)),
        origin=Origin(xyz=(-0.118, 0.0, 0.0)),
        material=graphite,
        name="rear_handle",
    )
    base.visual(
        Box((0.034, 0.040, 0.018)),
        origin=Origin(xyz=(-0.046, 0.0, 0.0)),
        material=graphite,
        name="handle_boss",
    )
    _add_clevis_yoke(base, x=0.0, material=brushed_steel, prefix="base")

    proximal = model.part("proximal_link")
    _add_phalanx(
        proximal,
        link_length=0.180,
        material=brushed_steel,
        barrel_material=dark_barrels,
        has_distal_yoke=True,
    )

    middle = model.part("middle_link")
    _add_phalanx(
        middle,
        link_length=0.150,
        material=brushed_steel,
        barrel_material=dark_barrels,
        has_distal_yoke=True,
    )

    distal = model.part("distal_link")
    _add_phalanx(
        distal,
        link_length=0.126,
        material=brushed_steel,
        barrel_material=dark_barrels,
        has_distal_yoke=False,
        fingertip_mesh=pad_mesh,
        pad_material=soft_pad,
        ridge_material=pad_ridge,
    )

    hinge_limits = MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.30)
    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=hinge_limits,
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=hinge_limits,
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=hinge_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_mount")
    proximal = object_model.get_part("proximal_link")
    middle = object_model.get_part("middle_link")
    distal = object_model.get_part("distal_link")
    joints = [
        object_model.get_articulation("base_to_proximal"),
        object_model.get_articulation("proximal_to_middle"),
        object_model.get_articulation("middle_to_distal"),
    ]

    def _allow_pin_capture(parent, child, *, pin_name: str, check_prefix: str) -> None:
        ctx.allow_overlap(
            parent,
            child,
            elem_a=pin_name,
            elem_b="proximal_barrel",
            reason=(
                "The hinge pin is intentionally captured concentrically inside "
                "the child's visible hinge barrel."
            ),
        )
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem=pin_name,
            outer_elem="proximal_barrel",
            margin=0.0,
            name=f"{check_prefix} pin centered in barrel",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a=pin_name,
            elem_b="proximal_barrel",
            min_overlap=0.020,
            name=f"{check_prefix} pin spans barrel width",
        )

    _allow_pin_capture(base, proximal, pin_name="base_pin", check_prefix="base hinge")
    _allow_pin_capture(proximal, middle, pin_name="distal_pin", check_prefix="middle hinge")
    _allow_pin_capture(middle, distal, pin_name="distal_pin", check_prefix="distal hinge")

    ctx.check(
        "three revolute bending joints",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "hinges share one side-to-side axis",
        all(abs(j.axis[1]) > 0.99 and abs(j.axis[0]) < 1e-6 and abs(j.axis[2]) < 1e-6 for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )
    ctx.check(
        "joint limits bend in one positive direction",
        all(
            j.motion_limits is not None
            and j.motion_limits.lower == 0.0
            and j.motion_limits.upper is not None
            and j.motion_limits.upper >= 1.2
            for j in joints
        ),
        details="All three knuckles should fold from straight to a realistic positive bend.",
    )

    pad_aabb = ctx.part_element_world_aabb(distal, elem="fingertip_pad")
    spar_aabb = ctx.part_element_world_aabb(distal, elem="slender_spar")
    ctx.check(
        "spatulate pad wider than phalanx",
        pad_aabb is not None
        and spar_aabb is not None
        and (pad_aabb[1][1] - pad_aabb[0][1]) > (spar_aabb[1][1] - spar_aabb[0][1]) * 2.5,
        details=f"pad_aabb={pad_aabb}, spar_aabb={spar_aabb}",
    )

    rest_pad_aabb = ctx.part_element_world_aabb(distal, elem="fingertip_pad")
    rest_z = None if rest_pad_aabb is None else (rest_pad_aabb[0][2] + rest_pad_aabb[1][2]) * 0.5
    with ctx.pose({joints[0]: 0.55, joints[1]: 0.55, joints[2]: 0.55}):
        bent_pad_aabb = ctx.part_element_world_aabb(distal, elem="fingertip_pad")
        bent_z = None if bent_pad_aabb is None else (bent_pad_aabb[0][2] + bent_pad_aabb[1][2]) * 0.5
    ctx.check(
        "positive joint motion curls fingertip upward",
        rest_z is not None and bent_z is not None and bent_z > rest_z + 0.12,
        details=f"rest_z={rest_z}, bent_z={bent_z}",
    )

    return ctx.report()


object_model = build_object_model()
