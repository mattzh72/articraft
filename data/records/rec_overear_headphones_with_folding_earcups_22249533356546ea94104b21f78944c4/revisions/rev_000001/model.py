from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_boom_headset")

    matte_black = model.material("matte_black", rgba=(0.015, 0.015, 0.018, 1.0))
    soft_leather = model.material("soft_leather", rgba=(0.025, 0.023, 0.022, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.060, 0.066, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.45, 0.47, 0.50, 1.0))
    microphone_foam = model.material("microphone_foam", rgba=(0.006, 0.006, 0.006, 1.0))
    speaker_fabric = model.material("speaker_fabric", rgba=(0.010, 0.011, 0.012, 1.0))

    def cyl_y(radius: float, length: float) -> Cylinder:
        # SDK cylinders are local-Z aligned; rotate the visual to put the pin axis on Y.
        return Cylinder(radius=radius, length=length)

    def ellipse_shell(name: str):
        return mesh_from_geometry(
            SphereGeometry(1.0, width_segments=48, height_segments=24).scale(0.027, 0.047, 0.066),
            name,
        )

    def oval_pad(name: str):
        pad = TorusGeometry(radius=0.034, tube=0.008, radial_segments=24, tubular_segments=64)
        pad.rotate_y(pi / 2.0).scale(1.0, 1.10, 1.45)
        return mesh_from_geometry(pad, name)

    def oval_disc(name: str, y_radius: float, z_radius: float, thickness: float):
        disc = CylinderGeometry(1.0, thickness, radial_segments=48, closed=True)
        disc.scale(z_radius, y_radius, 1.0).rotate_y(pi / 2.0)
        return mesh_from_geometry(disc, name)

    def mic_capsule(name: str):
        cap = CapsuleGeometry(radius=0.0075, length=0.024, radial_segments=24, height_segments=8)
        cap.rotate_x(pi / 2.0)
        return mesh_from_geometry(cap, name)

    def boom_tube(name: str):
        boom = tube_from_spline_points(
            [
                (-0.004, 0.000, 0.000),
                (-0.006, -0.045, -0.030),
                (-0.002, -0.095, -0.065),
                (0.010, -0.145, -0.078),
            ],
            radius=0.0035,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        )
        return mesh_from_geometry(boom, name)

    headband = model.part("headband")
    outer_band = tube_from_spline_points(
        [
            (-0.145, 0.000, 0.045),
            (-0.140, 0.000, 0.110),
            (-0.095, 0.000, 0.202),
            (0.000, 0.000, 0.245),
            (0.095, 0.000, 0.202),
            (0.140, 0.000, 0.110),
            (0.145, 0.000, 0.045),
        ],
        radius=0.010,
        samples_per_segment=20,
        radial_segments=20,
        cap_ends=True,
    )
    headband.visual(mesh_from_geometry(outer_band, "arched_headband"), material=satin_metal, name="arched_band")
    cushion = tube_from_spline_points(
        [
            (-0.075, 0.000, 0.204),
            (-0.035, 0.000, 0.222),
            (0.000, 0.000, 0.229),
            (0.035, 0.000, 0.222),
            (0.075, 0.000, 0.204),
        ],
        radius=0.014,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    headband.visual(mesh_from_geometry(cushion, "head_cushion"), material=soft_leather, name="head_pad")
    for side, x in (("left", -0.145), ("right", 0.145)):
        headband.visual(
            cyl_y(0.012, 0.020),
            origin=Origin(xyz=(x, 0.0, 0.045), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name=f"{side}_hinge_knuckle",
        )

    def add_yoke(side: str):
        yoke = model.part(f"{side}_yoke")
        # Two separated hinge knuckles interleave around the fixed headband knuckle.
        for y in (-0.0165, 0.0165):
            yoke.visual(
                cyl_y(0.012, 0.014),
                origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material=satin_metal,
                name=f"fold_knuckle_{0 if y < 0 else 1}",
            )
            yoke.visual(
                Box((0.014, 0.016, 0.026)),
                origin=Origin(xyz=(0.0, y, -0.018)),
                material=satin_metal,
                name=f"hinge_cheek_{0 if y < 0 else 1}",
            )
        yoke.visual(
            Box((0.016, 0.118, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.025)),
            material=satin_metal,
            name="top_bridge",
        )
        for y in (-0.055, 0.055):
            yoke.visual(
                Box((0.014, 0.010, 0.135)),
                origin=Origin(xyz=(0.0, y, -0.0855)),
                material=satin_metal,
                name=f"yoke_arm_{0 if y < 0 else 1}",
            )
            yoke.visual(
                cyl_y(0.006, 0.010),
                origin=Origin(xyz=(0.0, y * 0.91, -0.115), rpy=(pi / 2.0, 0.0, 0.0)),
                material=satin_metal,
                name=f"cup_trunnion_{0 if y < 0 else 1}",
            )
        return yoke

    left_yoke = add_yoke("left")
    right_yoke = add_yoke("right")

    def add_cup(side: str, inward_sign: float, with_mic_socket: bool = False):
        cup = model.part(f"{side}_cup")
        cup.visual(ellipse_shell(f"{side}_cup_shell"), material=dark_plastic, name="shell")
        cup.visual(
            oval_pad(f"{side}_ear_pad"),
            origin=Origin(xyz=(0.024 * inward_sign, 0.0, -0.004)),
            material=soft_leather,
            name="ear_cushion",
        )
        cup.visual(
            oval_disc(f"{side}_speaker_cloth", y_radius=0.026, z_radius=0.040, thickness=0.003),
            origin=Origin(xyz=(0.030 * inward_sign, 0.0, -0.004)),
            material=speaker_fabric,
            name="speaker_cloth",
        )
        cup.visual(
            oval_disc(f"{side}_outer_cap", y_radius=0.034, z_radius=0.050, thickness=0.006),
            origin=Origin(xyz=(-0.029 * inward_sign, 0.0, 0.004)),
            material=matte_black,
            name="outer_cap",
        )
        for y in (-0.047, 0.047):
            cup.visual(
                cyl_y(0.008, 0.004),
                origin=Origin(xyz=(0.0, y, -0.001), rpy=(pi / 2.0, 0.0, 0.0)),
                material=satin_metal,
                name=f"bearing_pad_{0 if y < 0 else 1}",
            )
        if with_mic_socket:
            cup.visual(
                Cylinder(radius=0.011, length=0.010),
                origin=Origin(xyz=(-0.033, -0.034, 0.008), rpy=(0.0, pi / 2.0, 0.0)),
                material=satin_metal,
                name="microphone_socket",
            )
        return cup

    left_cup = add_cup("left", inward_sign=1.0, with_mic_socket=True)
    right_cup = add_cup("right", inward_sign=-1.0, with_mic_socket=False)

    microphone = model.part("microphone")
    microphone.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="pivot_button",
    )
    microphone.visual(boom_tube("microphone_boom"), material=matte_black, name="boom")
    microphone.visual(
        mic_capsule("microphone_capsule"),
        origin=Origin(xyz=(0.012, -0.151, -0.078), rpy=(0.0, 0.0, -0.25)),
        material=microphone_foam,
        name="capsule",
    )

    model.articulation(
        "left_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.145, 0.0, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "right_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.145, 0.0, 0.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "left_cup_swivel",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "right_cup_swivel",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "microphone_pivot",
        ArticulationType.REVOLUTE,
        parent=left_cup,
        child=microphone,
        origin=Origin(xyz=(-0.041, -0.034, 0.008)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")
    microphone = object_model.get_part("microphone")
    left_fold = object_model.get_articulation("left_fold_hinge")
    right_fold = object_model.get_articulation("right_fold_hinge")
    left_swivel = object_model.get_articulation("left_cup_swivel")
    right_swivel = object_model.get_articulation("right_cup_swivel")
    mic_pivot = object_model.get_articulation("microphone_pivot")

    ctx.expect_contact(
        headband,
        left_yoke,
        elem_a="left_hinge_knuckle",
        elem_b="fold_knuckle_0",
        contact_tol=0.006,
        name="left folding hinge is physically adjacent",
    )
    ctx.expect_contact(
        headband,
        right_yoke,
        elem_a="right_hinge_knuckle",
        elem_b="fold_knuckle_1",
        contact_tol=0.006,
        name="right folding hinge is physically adjacent",
    )
    ctx.expect_overlap(left_cup, left_yoke, axes="z", min_overlap=0.020, name="left cup sits inside yoke height")
    ctx.expect_overlap(right_cup, right_yoke, axes="z", min_overlap=0.020, name="right cup sits inside yoke height")
    ctx.expect_contact(
        left_cup,
        microphone,
        elem_a="microphone_socket",
        elem_b="pivot_button",
        contact_tol=0.004,
        name="boom microphone pivot is seated on cup shell",
    )

    left_rest = ctx.part_world_position(left_cup)
    right_rest = ctx.part_world_position(right_cup)
    with ctx.pose({left_fold: 1.0, right_fold: 1.0}):
        left_folded = ctx.part_world_position(left_cup)
        right_folded = ctx.part_world_position(right_cup)
    ctx.check(
        "fold hinges swing both cups inward",
        left_rest is not None
        and right_rest is not None
        and left_folded is not None
        and right_folded is not None
        and left_folded[0] > left_rest[0] + 0.040
        and right_folded[0] < right_rest[0] - 0.040,
        details=f"rest L/R={left_rest}/{right_rest}, folded L/R={left_folded}/{right_folded}",
    )

    def elem_center_z(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    left_cap_rest_z = elem_center_z(left_cup, "outer_cap")
    right_cap_rest_z = elem_center_z(right_cup, "outer_cap")
    with ctx.pose({left_swivel: 0.35, right_swivel: 0.35}):
        left_cap_swiveled_z = elem_center_z(left_cup, "outer_cap")
        right_cap_swiveled_z = elem_center_z(right_cup, "outer_cap")
    ctx.check(
        "earcups rotate on their yoke trunnions",
        left_cap_rest_z is not None
        and right_cap_rest_z is not None
        and left_cap_swiveled_z is not None
        and right_cap_swiveled_z is not None
        and abs(left_cap_swiveled_z - left_cap_rest_z) > 0.006
        and abs(right_cap_swiveled_z - right_cap_rest_z) > 0.006,
        details=(
            f"left_z={left_cap_rest_z}->{left_cap_swiveled_z}, "
            f"right_z={right_cap_rest_z}->{right_cap_swiveled_z}"
        ),
    )

    mic_rest_z = elem_center_z(microphone, "capsule")
    with ctx.pose({mic_pivot: 1.0}):
        mic_raised_z = elem_center_z(microphone, "capsule")
    ctx.check(
        "boom microphone rotates upward",
        mic_rest_z is not None and mic_raised_z is not None and mic_raised_z > mic_rest_z + 0.020,
        details=f"rest_z={mic_rest_z}, raised_z={mic_raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
