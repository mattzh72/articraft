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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_closing_push_button_faucet")

    polished_chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    shadow_chrome = model.material("shadow_chrome", rgba=(0.62, 0.65, 0.69, 1.0))
    satin_button = model.material("satin_button", rgba=(0.78, 0.79, 0.81, 1.0))

    faucet_body = model.part("faucet_body")

    deck_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.165, 0.060, 0.028), 0.006),
        "deck_plate",
    )
    faucet_body.visual(
        deck_plate_mesh,
        origin=Origin(xyz=(0.010, 0.0, 0.003)),
        material=polished_chrome,
        name="deck_plate",
    )

    pedestal_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.026, 0.000),
                (0.030, 0.003),
                (0.028, 0.008),
                (0.024, 0.020),
                (0.022, 0.040),
                (0.022, 0.060),
            ],
            segments=64,
        ),
        "pedestal_body",
    )
    faucet_body.visual(
        pedestal_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=polished_chrome,
        name="pedestal_body",
    )

    guide_bezel_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.022, 0.060),
                (0.022, 0.086),
                (0.021, 0.098),
            ],
            [
                (0.0108, 0.060),
                (0.0108, 0.086),
                (0.0168, 0.086),
                (0.0168, 0.098),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "guide_bezel",
    )
    faucet_body.visual(
        guide_bezel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=polished_chrome,
        name="guide_bezel",
    )

    faucet_body.visual(
        Cylinder(radius=0.0145, length=0.016),
        origin=Origin(xyz=(0.019, 0.0, 0.081), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_chrome,
        name="spout_boss",
    )
    faucet_body.visual(
        Cylinder(radius=0.012, length=0.082),
        origin=Origin(xyz=(0.061, 0.0, 0.081), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_chrome,
        name="spout_tube",
    )
    faucet_body.visual(
        Cylinder(radius=0.0105, length=0.016),
        origin=Origin(xyz=(0.110, 0.0, 0.081), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shadow_chrome,
        name="spout_tip",
    )
    faucet_body.visual(
        Cylinder(radius=0.0065, length=0.005),
        origin=Origin(xyz=(0.114, 0.0, 0.0680)),
        material=shadow_chrome,
        name="aerator_lip",
    )
    faucet_body.inertial = Inertial.from_geometry(
        Box((0.190, 0.065, 0.110)),
        mass=1.8,
        origin=Origin(xyz=(0.020, 0.0, 0.055)),
    )

    push_button = model.part("push_button")
    plunger_head_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0156, 0.0075),
                (0.0188, 0.0095),
                (0.0208, 0.0125),
                (0.0202, 0.0165),
                (0.0170, 0.0200),
                (0.0105, 0.0225),
                (0.0000, 0.0235),
            ],
            segments=64,
        ),
        "plunger_head",
    )
    push_button.visual(
        Cylinder(radius=0.0072, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=shadow_chrome,
        name="button_stem",
    )
    push_button.visual(
        Cylinder(radius=0.0156, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_button,
        name="button_core",
    )
    push_button.visual(
        Cylinder(radius=0.0102, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=satin_button,
        name="button_neck",
    )
    push_button.visual(
        plunger_head_mesh,
        material=satin_button,
        name="button_head",
    )
    push_button.inertial = Inertial.from_geometry(
        Box((0.042, 0.042, 0.050)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
    )

    model.articulation(
        "button_press",
        ArticulationType.PRISMATIC,
        parent=faucet_body,
        child=push_button,
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
        motion_properties=MotionProperties(damping=18.0, friction=1.0),
        meta={
            "mechanism": "self_closing_push_button",
            "auto_return": "timed_spring_return",
        },
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    faucet_body = object_model.get_part("faucet_body")
    push_button = object_model.get_part("push_button")
    button_press = object_model.get_articulation("button_press")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_isolated_part(
        push_button,
        reason=(
            "The push button is a clearanced plunger guided by the faucet bezel; "
            "it is intentionally supported by a running fit rather than rest-pose surface contact."
        ),
    )

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

    ctx.expect_gap(
        push_button,
        faucet_body,
        axis="z",
        positive_elem="button_core",
        negative_elem="guide_bezel",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="button core starts flush with the top bezel",
    )
    ctx.expect_overlap(
        push_button,
        faucet_body,
        axes="xy",
        elem_a="button_core",
        elem_b="guide_bezel",
        min_overlap=0.030,
        name="button core is centered over the round pedestal top",
    )
    ctx.expect_within(
        push_button,
        faucet_body,
        axes="xy",
        inner_elem="button_stem",
        outer_elem="guide_bezel",
        margin=0.0,
        name="button stem stays centered within the guide bezel footprint",
    )
    ctx.expect_overlap(
        push_button,
        faucet_body,
        axes="z",
        elem_a="button_stem",
        elem_b="guide_bezel",
        min_overlap=0.025,
        name="button stem is retained inside the bezel guide at rest",
    )

    body_aabb = ctx.part_world_aabb(faucet_body)
    deck_plate_aabb = ctx.part_element_world_aabb(faucet_body, elem="deck_plate")
    pedestal_aabb = ctx.part_element_world_aabb(faucet_body, elem="pedestal_body")
    spout_tube_aabb = ctx.part_element_world_aabb(faucet_body, elem="spout_tube")
    spout_tip_aabb = ctx.part_element_world_aabb(faucet_body, elem="spout_tip")

    body_dims_ok = False
    spout_layout_ok = False
    if body_aabb is not None:
        dx = body_aabb[1][0] - body_aabb[0][0]
        dy = body_aabb[1][1] - body_aabb[0][1]
        dz = body_aabb[1][2] - body_aabb[0][2]
        body_dims_ok = 0.160 <= dx <= 0.205 and 0.055 <= dy <= 0.075 and 0.095 <= dz <= 0.120
        ctx.check(
            "faucet overall proportions are plausible",
            body_dims_ok,
            details=f"dims=({dx:.4f}, {dy:.4f}, {dz:.4f})",
        )

    if (
        deck_plate_aabb is not None
        and pedestal_aabb is not None
        and spout_tube_aabb is not None
        and spout_tip_aabb is not None
    ):
        spout_layout_ok = (
            spout_tube_aabb[0][2] >= deck_plate_aabb[1][2] + 0.060
            and spout_tip_aabb[1][0] >= pedestal_aabb[1][0] + 0.065
            and spout_tube_aabb[1][0] <= pedestal_aabb[1][0] + 0.105
        )
        ctx.check(
            "short straight spout projects forward above the deck plate",
            spout_layout_ok,
            details=(
                f"deck_top={deck_plate_aabb[1][2]:.4f}, "
                f"pedestal_front={pedestal_aabb[1][0]:.4f}, "
                f"spout_min_z={spout_tube_aabb[0][2]:.4f}, "
                f"tip_front={spout_tip_aabb[1][0]:.4f}"
            ),
        )

    upper = 0.0
    if button_press.motion_limits is not None and button_press.motion_limits.upper is not None:
        upper = button_press.motion_limits.upper
    rest_pos = ctx.part_world_position(push_button)
    pressed_pos = None
    with ctx.pose({button_press: upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="pressed button clears the faucet body")
        ctx.expect_overlap(
            push_button,
            faucet_body,
            axes="z",
            elem_a="button_stem",
            elem_b="guide_bezel",
            min_overlap=0.020,
            name="pressed button stem remains retained inside the guide",
        )
        ctx.expect_gap(
            push_button,
            faucet_body,
            axis="z",
            positive_elem="button_head",
            negative_elem="guide_bezel",
            min_gap=0.002,
            max_gap=0.004,
            name="mushroom cap stays visibly proud when pressed",
        )
        pressed_pos = ctx.part_world_position(push_button)

    ctx.check(
        "button press moves downward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.003,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )
    ctx.check(
        "button articulation is authored as timed self-closing",
        button_press.meta.get("auto_return") == "timed_spring_return"
        and button_press.motion_properties is not None
        and (button_press.motion_properties.damping or 0.0) > 0.0,
        details=f"meta={button_press.meta}, motion_properties={button_press.motion_properties}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
