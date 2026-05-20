from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_candy_vending_machine")

    pedestal_black = model.material("pedestal_black", rgba=(0.14, 0.14, 0.15, 1.0))
    enamel_red = model.material("enamel_red", rgba=(0.70, 0.07, 0.10, 1.0))
    chrome = model.material("chrome", rgba=(0.76, 0.78, 0.82, 1.0))
    clear_acrylic = model.material("clear_acrylic", rgba=(0.77, 0.90, 0.98, 0.28))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))

    pedestal_foot = _mesh(
        "pedestal_foot",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.16, 0.0),
                (0.20, 0.01),
                (0.22, 0.03),
                (0.22, 0.06),
                (0.18, 0.08),
                (0.09, 0.09),
                (0.0, 0.09),
            ],
            segments=56,
        ),
    )
    reservoir_shell = _mesh(
        "reservoir_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.082, 0.00),
                (0.100, 0.02),
                (0.135, 0.08),
                (0.160, 0.15),
                (0.148, 0.24),
                (0.110, 0.29),
                (0.085, 0.30),
            ],
            inner_profile=[
                (0.076, 0.00),
                (0.094, 0.02),
                (0.128, 0.08),
                (0.152, 0.15),
                (0.140, 0.24),
                (0.103, 0.29),
                (0.079, 0.30),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )

    body = model.part("machine_body")
    body.visual(pedestal_foot, material=pedestal_black, name="pedestal_foot")
    body.visual(
        Cylinder(radius=0.055, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=pedestal_black,
        name="pedestal_column",
    )
    body.visual(
        Cylinder(radius=0.10, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
        material=chrome,
        name="lower_collar",
    )
    body.visual(
        Box((0.28, 0.24, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        material=enamel_red,
        name="coin_box_shell",
    )
    body.visual(
        Box((0.012, 0.18, 0.16)),
        origin=Origin(xyz=(0.146, 0.0, 1.01)),
        material=chrome,
        name="front_service_plate",
    )
    body.visual(
        Box((0.016, 0.086, 0.020)),
        origin=Origin(xyz=(0.148, 0.0, 1.075)),
        material=chrome,
        name="coin_slot",
    )
    body.visual(
        Cylinder(radius=0.048, length=0.012),
        origin=Origin(xyz=(0.146, 0.0, 0.998), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="knob_bezel",
    )
    body.visual(
        Box((0.12, 0.09, 0.012)),
        origin=Origin(xyz=(0.20, 0.0, 0.894)),
        material=chrome,
        name="chute_floor",
    )
    body.visual(
        Box((0.085, 0.09, 0.012)),
        origin=Origin(xyz=(0.1825, 0.0, 0.940)),
        material=chrome,
        name="chute_roof",
    )
    body.visual(
        Box((0.085, 0.010, 0.056)),
        origin=Origin(xyz=(0.1825, 0.040, 0.917)),
        material=chrome,
        name="chute_side_left",
    )
    body.visual(
        Box((0.085, 0.010, 0.056)),
        origin=Origin(xyz=(0.1825, -0.040, 0.917)),
        material=chrome,
        name="chute_side_right",
    )
    body.visual(
        Cylinder(radius=0.092, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.14)),
        material=chrome,
        name="upper_collar",
    )
    body.visual(
        reservoir_shell,
        origin=Origin(xyz=(0.0, 0.0, 1.17)),
        material=clear_acrylic,
        name="reservoir_shell",
    )
    body.visual(
        Cylinder(radius=0.092, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 1.479)),
        material=chrome,
        name="lid",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 1.500)),
        material=chrome,
        name="lid_cap",
    )

    knob = model.part("dispense_knob")
    knob.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="axle",
    )
    knob.visual(
        Cylinder(radius=0.040, length=0.013),
        origin=Origin(xyz=(0.0245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="wheel",
    )
    knob.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.041, 0.028, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="finger_post",
    )
    knob.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.051, 0.028, 0.0)),
        material=grip_black,
        name="finger_grip",
    )

    model.articulation(
        "knob_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.152, 0.0, 0.998)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("machine_body")
    knob = object_model.get_part("dispense_knob")
    knob_joint = object_model.get_articulation("knob_rotation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        knob,
        body,
        elem_a="axle",
        elem_b="knob_bezel",
        contact_tol=0.0005,
        name="knob axle seats on bezel",
    )
    ctx.expect_origin_gap(
        knob,
        body,
        axis="x",
        min_gap=0.14,
        max_gap=0.17,
        name="knob is mounted on the front face",
    )

    body_aabb = ctx.part_world_aabb(body)
    shell_aabb = ctx.part_element_world_aabb(body, elem="coin_box_shell")
    reservoir_aabb = ctx.part_element_world_aabb(body, elem="reservoir_shell")
    slot_aabb = ctx.part_element_world_aabb(body, elem="coin_slot")
    chute_aabb = ctx.part_element_world_aabb(body, elem="chute_roof")
    grip_rest_aabb = ctx.part_element_world_aabb(knob, elem="finger_grip")

    body_height = None if body_aabb is None else body_aabb[1][2] - body_aabb[0][2]
    shell_top = None if shell_aabb is None else shell_aabb[1][2]
    reservoir_bottom = None if reservoir_aabb is None else reservoir_aabb[0][2]
    reservoir_top = None if reservoir_aabb is None else reservoir_aabb[1][2]
    slot_center = _aabb_center(slot_aabb)
    chute_center = _aabb_center(chute_aabb)
    grip_center_rest = _aabb_center(grip_rest_aabb)

    ctx.check(
        "machine reads as freestanding full-height unit",
        body_height is not None and 1.45 <= body_height <= 1.60,
        details=f"body_height={body_height}",
    )
    ctx.check(
        "clear reservoir sits above the coin box",
        shell_top is not None
        and reservoir_bottom is not None
        and reservoir_top is not None
        and reservoir_bottom >= shell_top - 0.002
        and reservoir_top >= 1.45,
        details=(
            f"coin_box_top={shell_top}, reservoir_bottom={reservoir_bottom}, "
            f"reservoir_top={reservoir_top}"
        ),
    )
    ctx.check(
        "coin slot, knob, and chute stack on the front face",
        slot_center is not None
        and grip_center_rest is not None
        and chute_center is not None
        and slot_center[0] > 0.14
        and grip_center_rest[0] > 0.18
        and chute_center[0] > 0.14
        and slot_center[2] > grip_center_rest[2] > chute_center[2],
        details=(
            f"slot_center={slot_center}, grip_center={grip_center_rest}, "
            f"chute_center={chute_center}"
        ),
    )

    with ctx.pose({knob_joint: math.pi / 2.0}):
        grip_quarter_aabb = ctx.part_element_world_aabb(knob, elem="finger_grip")

    grip_center_quarter = _aabb_center(grip_quarter_aabb)
    ctx.check(
        "knob quarter turn swings the grip upward",
        grip_center_rest is not None
        and grip_center_quarter is not None
        and grip_center_quarter[2] > grip_center_rest[2] + 0.015
        and abs(grip_center_quarter[1]) < abs(grip_center_rest[1]),
        details=f"rest={grip_center_rest}, quarter_turn={grip_center_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
