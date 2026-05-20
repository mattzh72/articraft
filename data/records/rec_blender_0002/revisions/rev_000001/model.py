from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_serve_blender", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.16, 0.17, 0.18, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.72, 0.82, 0.90, 0.28))
    stainless = model.material("stainless", rgba=(0.70, 0.72, 0.75, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    seal_black = model.material("seal_black", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    base_body_mesh = _save_mesh(
        "blender_base_body.obj",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.066, 0.0),
                (0.081, 0.008),
                (0.093, 0.028),
                (0.101, 0.070),
                (0.104, 0.118),
                (0.098, 0.136),
                (0.085, 0.148),
                (0.040, 0.148),
                (0.040, 0.124),
                (0.0, 0.124),
            ],
            segments=80,
        ),
    )
    base.visual(base_body_mesh, material=charcoal, name="body_shell")
    base.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
        material=stainless,
        name="bayonet_collar",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
        material=soft_gray,
        name="drive_coupler_socket",
    )
    base.visual(
        Cylinder(radius=0.076, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=matte_black,
        name="base_trim",
    )
    for index, angle_deg in enumerate((0.0, 120.0, 240.0)):
        angle = math.radians(angle_deg)
        base.visual(
            Box((0.015, 0.026, 0.008)),
            origin=Origin(
                xyz=(0.060 * math.cos(angle), 0.060 * math.sin(angle), 0.144),
                rpy=(0.0, 0.0, angle),
            ),
            material=stainless,
            name=f"bayonet_receiver_{index}",
        )
        base.visual(
            Box((0.010, 0.010, 0.006)),
            origin=Origin(
                xyz=(0.070 * math.cos(angle + 0.12), 0.070 * math.sin(angle + 0.12), 0.146),
                rpy=(0.0, 0.0, angle + 0.12),
            ),
            material=soft_gray,
            name=f"bayonet_stop_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.104, length=0.148),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
    )

    actuator = model.part("actuator")
    actuator.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=matte_black,
        name="guide_collar",
    )
    actuator.visual(
        Cylinder(radius=0.033, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=soft_gray,
        name="push_plate",
    )
    actuator.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=stainless,
        name="push_cap",
    )
    actuator.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.028),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    jar = model.part("jar")
    jar.visual(
        Cylinder(radius=0.056, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stainless,
        name="collar_band",
    )
    jar.visual(
        Cylinder(radius=0.047, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=stainless,
        name="lower_shoulder",
    )
    jar.visual(
        Cylinder(radius=0.045, length=0.248),
        origin=Origin(xyz=(0.0, 0.0, 0.149)),
        material=dark_glass,
        name="body_shell",
    )
    jar.visual(
        Cylinder(radius=0.048, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.278)),
        material=soft_gray,
        name="jar_rim",
    )
    for index, angle_deg in enumerate((0.0, 120.0, 240.0)):
        angle = math.radians(angle_deg)
        jar.visual(
            Box((0.015, 0.024, 0.008)),
            origin=Origin(
                xyz=(0.060 * math.cos(angle), 0.060 * math.sin(angle), 0.004),
                rpy=(0.0, 0.0, angle),
            ),
            material=stainless,
            name=f"bayonet_lug_{index}",
        )
    jar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.294),
        mass=0.48,
        origin=Origin(xyz=(0.0, 0.0, 0.147)),
    )

    lid = model.part("lid")
    lid_shell_mesh = _save_mesh(
        "blender_lid_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.050, 0.0),
                (0.050, 0.012),
                (0.046, 0.018),
                (0.034, 0.024),
                (0.018, 0.024),
            ],
            [
                (0.042, 0.0),
                (0.042, 0.010),
                (0.038, 0.016),
                (0.028, 0.020),
                (0.016, 0.020),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    lid.visual(lid_shell_mesh, material=seal_black, name="lid_shell")
    lid.visual(
        Cylinder(radius=0.042, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=matte_black,
        name="seal_band",
    )
    lid.visual(
        Box((0.014, 0.018, 0.020)),
        origin=Origin(xyz=(-0.051, 0.0, 0.001)),
        material=matte_black,
        name="left_tab",
    )
    lid.visual(
        Box((0.006, 0.020, 0.006)),
        origin=Origin(xyz=(-0.057, 0.0, -0.006)),
        material=matte_black,
        name="left_hook",
    )
    lid.visual(
        Box((0.014, 0.018, 0.020)),
        origin=Origin(xyz=(0.051, 0.0, 0.001)),
        material=matte_black,
        name="right_tab",
    )
    lid.visual(
        Box((0.006, 0.020, 0.006)),
        origin=Origin(xyz=(0.057, 0.0, -0.006)),
        material=matte_black,
        name="right_hook",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.034),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=soft_gray,
        name="button_skirt",
    )
    release_button.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=stainless,
        name="button_cap",
    )
    release_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.022),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    model.articulation(
        "base_to_actuator",
        ArticulationType.PRISMATIC,
        parent=base,
        child=actuator,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )
    model.articulation(
        "base_to_jar",
        ArticulationType.FIXED,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.FIXED,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.286)),
    )
    model.articulation(
        "lid_to_release_button",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=release_button,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    actuator = object_model.get_part("actuator")
    jar = object_model.get_part("jar")
    lid = object_model.get_part("lid")
    release_button = object_model.get_part("release_button")
    base_to_actuator = object_model.get_articulation("base_to_actuator")
    lid_to_release_button = object_model.get_articulation("lid_to_release_button")

    base_body = base.get_visual("body_shell")
    bayonet_collar = base.get_visual("bayonet_collar")
    receiver_0 = base.get_visual("bayonet_receiver_0")
    receiver_1 = base.get_visual("bayonet_receiver_1")
    receiver_2 = base.get_visual("bayonet_receiver_2")
    guide_collar = actuator.get_visual("guide_collar")
    collar_band = jar.get_visual("collar_band")
    jar_body = jar.get_visual("body_shell")
    jar_rim = jar.get_visual("jar_rim")
    lug_0 = jar.get_visual("bayonet_lug_0")
    lug_1 = jar.get_visual("bayonet_lug_1")
    lug_2 = jar.get_visual("bayonet_lug_2")
    lid_shell = lid.get_visual("lid_shell")
    seal_band = lid.get_visual("seal_band")
    left_tab = lid.get_visual("left_tab")
    right_tab = lid.get_visual("right_tab")
    left_hook = lid.get_visual("left_hook")
    right_hook = lid.get_visual("right_hook")
    button_skirt = release_button.get_visual("button_skirt")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        actuator,
        base,
        reason="push-to-blend plunger rides inside the recessed top guide of the motor base",
    )
    ctx.allow_overlap(
        release_button,
        lid,
        reason="center release button is intentionally nested inside the lid's guide aperture",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(jar, base, axes="xy", min_overlap=0.008)
    ctx.expect_within(jar, base, axes="xy")
    ctx.expect_gap(
        jar,
        base,
        axis="z",
        positive_elem=collar_band,
        negative_elem=bayonet_collar,
        max_gap=0.001,
        max_penetration=0.0,
        name="jar_collar_seats_on_base",
    )
    ctx.expect_overlap(
        jar,
        base,
        axes="xy",
        elem_a=lug_0,
        elem_b=receiver_0,
        min_overlap=0.0001,
        name="bayonet_lug_0_aligns_with_receiver",
    )
    ctx.expect_overlap(
        jar,
        base,
        axes="xy",
        elem_a=lug_1,
        elem_b=receiver_1,
        min_overlap=0.0001,
        name="bayonet_lug_1_aligns_with_receiver",
    )
    ctx.expect_overlap(
        jar,
        base,
        axes="xy",
        elem_a=lug_2,
        elem_b=receiver_2,
        min_overlap=0.0001,
        name="bayonet_lug_2_aligns_with_receiver",
    )
    ctx.expect_gap(
        jar,
        base,
        axis="z",
        positive_elem=lug_0,
        negative_elem=receiver_0,
        min_gap=0.0,
        max_gap=0.004,
        name="bayonet_lugs_hover_just_above_receivers",
    )

    ctx.expect_within(actuator, base, axes="xy")
    ctx.expect_origin_distance(
        actuator,
        base,
        axes="xy",
        max_dist=0.001,
        name="push_actuator_is_centered_on_base",
    )
    ctx.expect_contact(
        actuator,
        base,
        elem_a=guide_collar,
        elem_b=base_body,
        name="actuator_guided_in_base_recess",
    )

    ctx.expect_overlap(lid, jar, axes="xy", min_overlap=0.008)
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.25,
        name="jar_and_lid_form_tall_single_serve_stack",
    )
    ctx.expect_gap(
        lid,
        jar,
        axis="z",
        positive_elem=seal_band,
        negative_elem=jar_rim,
        max_gap=0.001,
        max_penetration=1e-6,
        name="lid_seal_sits_on_jar_rim",
    )
    ctx.expect_overlap(
        lid,
        jar,
        axes="yz",
        elem_a=left_hook,
        elem_b=jar_rim,
        min_overlap=0.0001,
        name="left_snap_tab_reads_on_jar_side",
    )
    ctx.expect_overlap(
        lid,
        jar,
        axes="yz",
        elem_a=right_hook,
        elem_b=jar_rim,
        min_overlap=0.0001,
        name="right_snap_tab_reads_on_jar_side",
    )

    ctx.expect_within(release_button, lid, axes="xy")
    ctx.expect_overlap(
        release_button,
        lid,
        axes="xy",
        elem_a=button_skirt,
        elem_b=lid_shell,
        min_overlap=0.0001,
        name="release_button_centered_in_lid",
    )

    with ctx.pose({base_to_actuator: 0.004}):
        ctx.expect_within(actuator, base, axes="xy")
        ctx.expect_overlap(actuator, base, axes="xy", elem_a=guide_collar, elem_b=base_body, min_overlap=0.0001)

    with ctx.pose({lid_to_release_button: 0.003}):
        ctx.expect_within(release_button, lid, axes="xy")
        ctx.expect_overlap(
            release_button,
            lid,
            axes="xy",
            elem_a=button_skirt,
            elem_b=lid_shell,
            min_overlap=0.0001,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
